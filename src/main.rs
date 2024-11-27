#![no_std]
#![no_main]

pub mod heap;
pub mod pid;

extern crate alloc;

use core::cell::RefCell;
use core::cmp::{max, min};
use core::convert::Infallible;
use core::ops::Not;
use core::str::from_utf8;
use core::{future, time};
use alloc::rc::Rc;
use alloc::string::{String, ToString};
use alloc::vec::Vec;
use embassy_futures::join::{join3, join4, join5};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::{self, Receiver, Sender};
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use embedded_hal::delay::DelayNs as BDelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal_async::delay::{self, DelayNs as ADelayNs};
use embedded_hal_async::digital::Wait;
use embedded_io::*;
use esp_backtrace as _;
use esp_hal::dma::ReadBuffer;
use esp_hal::gpio::{Flex, GpioPin, Level, Output, OutputOpenDrain, Pull};
use esp_hal::mcpwm::operator::{Operator, PwmPin, PwmPinConfig};
use esp_hal::mcpwm::{McPwm, PeripheralClockConfig};
use esp_hal::time::now;
use esp_hal::timer::timg::{self, TimerGroup};
use esp_hal::uart::{self, Uart, UartRx};
use esp_hal::{
    delay::Delay,
    gpio::{self, Io},
    mcpwm::PwmPeripheral,
    prelude::*,
    time::Duration,
    xtensa_lx::timer::delay,
};
use esp_hal::{peripherals, prelude::*, Mode};
use esp_println::{logger, println};
use fugit::{HertzU32, MicrosDurationU32, Rate};
use heap::init_heap;
use log::{debug, info};
use pid::{Pid, PidConst, PidConsts};
use pipe::Pipeable;
use regex::Regex;

#[esp_hal_embassy::main]
async fn main(spawner: embassy_executor::Spawner) -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    init_heap();

    logger::init_logger_from_env();

    let timer_group_0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timer_group_0.timer0);

    println!("initializing heater");
    let heater_pin          = Output::new(io.pins.gpio18, Level::Low);
    let heater_value        = RefCell::new(0.8);
    let header_drive_task   = make_pwm(HertzU32::Hz(1), heater_pin, &heater_value);

    println!("initializing cooler");
    let cooler_pin = Output::new(io.pins.gpio19, Level::Low);
    let cooler_value = RefCell::new(0.2);
    let cooler_drive_task = make_pwm(
        HertzU32::Hz(1), 
        cooler_pin, 
        &cooler_value
    );

    let pid_freq        = HertzU32::Hz(10);
    let setpoint        = RefCell::new(24.0);
    let temp_value      = RefCell::new(24.0);

    println!("initializing thermometer");
    let dht_pin         = OutputOpenDrain::new(io.pins.gpio26, Level::Low, Pull::None);
    // let dht             = dht_embedded::Dht22::new(NoopInterruptControl, Delay::new(), dht_pin);
    let dht             = dht::dht22::Dht22::new(dht_pin, embassy_time::Delay);
    let temp_read_task  = make_temp_reader(&temp_value, dht, pid_freq.clone());

    let io_tasks = join3(header_drive_task, cooler_drive_task, temp_read_task);

    println!("initializing PID");
    let pid_consts = PidConsts {
        d: 0.0,
        p: 0.1,
        i: 0.001,
    };
    
    let pid_cycle_duration: MicrosDurationU32 = pid_freq.clone().into_duration();
    let pid = RefCell::new(Pid::new(pid_consts, (pid_cycle_duration.to_micros() as f32) / 1_000_000.0 ));
    let pid_task = make_pid_runner(&cooler_value, &setpoint, &temp_value, &pid, pid_freq);

    println!("initializing command reader tasks");
    let lines_channel = channel::Channel::new();
    let cmds_channel  = channel::Channel::new(); 

    let cmds_tasks = {
        let serial              = Uart::new(peripherals.UART0, io.pins.gpio3, io.pins.gpio1).unwrap() ;
        
        let uart_reader_task    = make_uart_reader(lines_channel.sender(), serial);
        let cmds_parser         = make_commands_parser(lines_channel.receiver(), cmds_channel.sender());
        let cmds_executor       = make_command_executor(
            cmds_channel.receiver(), 
            &setpoint, 
            &cooler_value, 
            &pid
        );
        join3(uart_reader_task, cmds_parser, cmds_executor, )
    };

    println!("running everything");

    let all_tasks = join3(io_tasks, pid_task, cmds_tasks);
    all_tasks.await;
    loop{}
}
pub async fn make_uart_reader<'a>(
    command_line_sender: Sender<'a, NoopRawMutex, String, 3>,
    mut uart_reader: Uart<'a, impl uart::Instance, impl Mode>
) {
    let mut buf = Vec::new();
    '_WAITING: loop{
        if uart_reader.read_ready().is_ok_and(|x| x){
            'READING: loop {
                match uart_reader.read_byte(){
                    Ok(byte) => {     buf.push(byte);
                    },
                    Err(nb::Error::WouldBlock) => break 'READING,
                    Err(nb::Error::Other(other_error)) => panic!("uart reader panicked, like your mom on a saturday night: {other_error:?}"),
                }
            }
            let mut last = None;
            // has read something, now we take it and send.
            'SENDING_LINES: for line in buf.split_inclusive_mut(|c| c == &b'\n') {
                let Ok(line_str) = from_utf8(line) else {
                    println!("invalid character found.");
                    continue 'SENDING_LINES;
                } ;
                if line_str.ends_with('\n').not(){
                    last = Some(line_str);
                    break;
                }
                command_line_sender.send(line_str.to_string()).await;
            }
            if let Some(last) = last{
                let mut last_bytes = last.to_string().into_bytes();
                buf.clear();
                buf.append(&mut last_bytes);
            }
        } else {
            Timer::after_millis(10).await;
        }
    }
}
pub async fn make_commands_parser<'a>(
    command_line_receiver: Receiver<'a, NoopRawMutex, String, 3>,
    commands_sender      : Sender<'a, NoopRawMutex, Command, 1>
) {
    let cmd_regex = Regex::new(r"(?P<target>(heater|setpoint|kp|ki|kd))[[:space:]]*:[[:space:]]*(?P<value>(?P<sign>\+|-)?(?P<integer>[[:digit:]]+)(\.(?P<decimal>[[:digit:]]+)?)?)").unwrap();
    loop{
        let cmd_line = command_line_receiver.receive().await;
        let Some(caps) = cmd_regex.captures(&cmd_line) else {
            println!("couldn't understand command: '{cmd_line}'");
            continue;
        };
        let value = caps.name("value").unwrap().as_str().parse::<f32>().unwrap();
        let cmd = match caps.name("target").unwrap().as_str().to_lowercase().as_str(){
            "heater" => Command::UpdateHeaterPower(value),
            "setpoint" => Command::UpdateSetpoint(value),
            "kp" => Command::UpdatePid{new_value: value, target: PidConst::P},
            "ki" => Command::UpdatePid{new_value: value, target: PidConst::I},
            "kd" => Command::UpdatePid{new_value: value, target: PidConst::D},
            _ => unreachable!() 
        };
        commands_sender.send(cmd).await;
    }
}

pub async fn make_command_executor<'a>(
    command_receiver: Receiver<'a, NoopRawMutex, Command, 1>,
    setpoint_cell   : &'a RefCell<f32>,
    heater_cell     : &'a RefCell<f32>,
    pid_cell        : &'a RefCell<Pid>
) {
    loop{
        let cmd= command_receiver.receive().await;
        match cmd{
            Command::UpdateSetpoint(new_setpoint) => {
                println!("updating setpoint: {new_setpoint}");
                *setpoint_cell.borrow_mut() = new_setpoint
            },
            Command::UpdateHeaterPower(new_heater_value) => {
                println!("updating heater power: {new_heater_value}");
                *heater_cell.borrow_mut() = new_heater_value;
            },
            Command::UpdatePid { new_value, target } => {
                println!("updating pid const. \n\t target:{target:?}, \n\tvalue: {new_value}\n");
                pid_cell.borrow_mut().consts.update_value(new_value, target);
            },
        }
    }
}

pub async fn make_pid_runner<'a>(
    x_cell      : &'a RefCell<f32>,
    setpoint    : &'a RefCell<f32>,
    y           : &'a RefCell<f32>,
    pid         : &'a RefCell<Pid>,
    frequency   : HertzU32,
) {
    let cycle_time: MicrosDurationU32 = frequency.into_duration();
    loop {
        let start = now();
        let cycle_end = start + cycle_time;
        let setpoint = setpoint.borrow().clone();
        println!("setpoint: {setpoint}\r");
        let y = y.borrow().clone();
        let error = y - setpoint;
        println!("error: {error}\r");
        let x= pid.borrow_mut().run_loop(error);
        let x = x.pipe(|x| f32::min(1.0, x)).pipe(|x| f32::max(0.0, x));
        println!("x: {x}\r");
        *x_cell.borrow_mut() = x;
        let time_to_wait = cycle_end - now();
        Timer::after_micros(time_to_wait.to_micros()) .await
    }
}
pub async fn make_temp_reader<'a, Pin, DhtDelay>(
    reading_output      : &'a RefCell<f32>,
    mut dht             : dht::dht22::Dht22<Pin, DhtDelay>,
    // mut dht             : Dht22<Infallible, NoopInterruptControl, DhtDelay, Pin>,
    reading_frequency   : HertzU32,
) 
    where 
        Pin     : OutputPin<Error=Infallible> + InputPin<Error=Infallible> , 
        DhtDelay: ADelayNs + BDelayNs
{
    let cycle_time = reading_frequency.into_duration();
    loop {
        let start = now();
        let end_time = start + cycle_time;
        // let temp = dht.read().unwrap().temperature();
        // let temp = dht.read_async().await.unwrap().temperature;
        let temp = 24.3;
        *reading_output.borrow_mut() = temp;
        println!("temp: {temp}");

        let time_to_wait = end_time - now();

        if time_to_wait >= Duration::from_ticks(0) {
            Timer::after_micros(time_to_wait.to_micros()).await;
        }
    }
}

pub async fn make_pwm<'a>(
    frequency   : HertzU32,
    mut pin     : impl OutputPin,
    value_cell  : &'a RefCell<f32>,
) {
    let cycle_time: MicrosDurationU32 = frequency.into_duration();

    loop {
        let value = value_cell.borrow().clone();

        let up_time = 
            value
            .pipe(|x| f32::min(1.0, x))
            .pipe(|x| f32::max(0.0, x))
            .pipe(|x| x * cycle_time.to_micros() as f32)
            .pipe(|x| x as u32)
            .pipe(MicrosDurationU32::micros);

        let down_time = cycle_time - up_time;

        pin.set_high().unwrap();
        Timer::after_micros(up_time.to_micros() as u64).await;
        pin.set_low().unwrap();
        Timer::after_micros(down_time.to_micros() as u64).await;
    }
}

pub enum Command {
    UpdateSetpoint(f32),
    UpdateHeaterPower(f32),
    UpdatePid { new_value: f32, target: PidConst },
}


pub struct ADelay;
impl ADelayNs for ADelay{
    async fn delay_ns(&mut self, ns: u32) {
        Timer::after_nanos(ns as u64).await
    }
    
    async fn delay_us(&mut self, us: u32) {
        Timer::after_micros(us as u64).await;    
    }
    
    async fn delay_ms(&mut self, mut ms: u32) {
        Timer::after_millis(ms as u64).await;
    }
}