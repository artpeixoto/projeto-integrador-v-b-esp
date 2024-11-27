
pub struct Pid {
    previous_input: f32,
    integrated: f32,

    timestep: f32,
    pub consts: PidConsts,
}


impl Pid {
    pub fn new(consts: PidConsts, timestep_sec: f32) -> Self {
        Self {
            previous_input	: 0.0,
            integrated		: 0.0,
            timestep		: timestep_sec,
            consts			: consts,
        }
    }
    pub fn run_loop(&mut self, input: f32) -> f32 {
        let difference = (input - self.previous_input) / self.timestep;
        let integrated = self.integrated + ((input + self.previous_input) / 2.0) * self.timestep;

        self.previous_input = input;
        self.integrated = integrated;

        let output =
            self.consts.d * difference + self.consts.i * integrated + self.consts.p * input;
        output
    }
}

pub struct PidConsts {
    pub p: f32,
    pub i: f32,
    pub d: f32,
}

impl PidConsts{
	pub fn update_value(&mut self, new_value: f32, target: PidConst){
		let target =  match target{
			PidConst::P => &mut self.p,
			PidConst::I => &mut self.i,
			PidConst::D => &mut self.d,
		};
		*target = new_value;
	}
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PidConst{ P, I, D, }

