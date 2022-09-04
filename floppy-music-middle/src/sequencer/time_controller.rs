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
        self.update_tick();
    }

    pub fn delta_micros(&self, delta: u32) -> u32 {
        self.tick_length * delta
    }

    pub fn set_ppq(&mut self, ppq: u32) {
        self.ppq = ppq;
        self.update_tick();
    }

    fn update_tick(&mut self) {
        self.tick_length = self.tempo / self.ppq;
    }
}
