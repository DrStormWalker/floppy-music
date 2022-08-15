#[derive(Copy, Clone, Debug)]
pub struct TimeController {
    tick_length: u32,
    tempo: u32,
    ppq: u32,
}
impl TimeController {
    pub fn new(tempo: u32, ppq: u32) -> Self {
        let mut controller = Self {
            tick_length: 0,
            tempo,
            ppq,
        };

        controller.set_tempo(tempo);

        controller
    }

    pub fn set_tempo(&mut self, tempo: u32) {
        self.tempo = tempo;
        self.tick_length = tempo / self.ppq;
    }

    pub fn delta_micros(&self, delta: u32) -> u32 {
        self.tick_length * delta
    }
}
