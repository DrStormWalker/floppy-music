use midly::{
    live::{LiveEvent, SystemCommon, SystemRealtime},
    MetaMessage, MidiMessage, Timing, Track, TrackEventKind,
};
use tokio::sync::mpsc;

use super::time_controller::TimeController;

pub struct MidiEngine<'a> {
    time_controller: TimeController,
    file: midly::Smf<'a>,
}
impl<'a> MidiEngine<'a> {
    pub fn new(file: midly::Smf<'a>) -> Self {
        let tempo = 0x07A120;

        let timing = match file.header.timing {
            Timing::Metrical(ppq) => ppq.as_int(),
            _ => unimplemented!("Only the Timing method Metrical is implemented"),
        };

        Self {
            time_controller: TimeController::new(tempo, timing as u32),
            file,
        }
    }

    pub fn play_to(&self, outputs: Vec<Option<mpsc::Sender<Vec<u8>>>>) -> MidiEngineInstance<'a> {
        MidiEngineInstance::new(self.time_controller, self.file.clone(), outputs)
    }
}

pub struct MidiEngineInstance<'a> {
    time_controller: TimeController,
    file: midly::Smf<'a>,
    outputs: Vec<Option<mpsc::Sender<Vec<u8>>>>,
}
impl<'a> MidiEngineInstance<'a> {
    pub fn new(
        time_controller: TimeController,
        file: midly::Smf<'a>,
        outputs: Vec<Option<mpsc::Sender<Vec<u8>>>>,
    ) -> Self {
        Self {
            time_controller,
            file,
            outputs,
        }
    }

    pub async fn start(&self) {
        let threads = self
            .file
            .tracks
            .iter()
            .enumerate()
            .zip(self.outputs.iter().map(|output| output.clone()))
            .filter_map(|((track_num, track), output)| {
                if let Some(output) = output {
                    Some(async move {
                        Self::thread(self.time_controller, output, track_num, track).await
                    })
                } else {
                    None
                }
            });

        futures::future::join_all(threads).await;
    }

    async fn thread(
        mut time_controller: TimeController,
        output: mpsc::Sender<Vec<u8>>,
        track_num: usize,
        track: &Track<'a>,
    ) {
        for event in track.iter() {
            tokio::time::sleep(tokio::time::Duration::from_micros(
                time_controller.delta_micros(event.delta.as_int()) as u64,
            ))
            .await;
            let event = event.kind;
            match event.as_live_event() {
                Some(event) => {
                    let mut buf = vec![];
                    event.write_std(&mut buf).unwrap();

                    output.send(buf).await.unwrap();
                }
                _ => {
                    if let TrackEventKind::Meta(msg) = event {
                        match msg {
                            MetaMessage::Tempo(tempo) => time_controller.set_tempo(tempo.as_int()),
                            _ => {}
                        }
                    } else {
                        unreachable!();
                    }
                }
            }
        }
    }

    pub async fn cleanup(&self) {
        let mut buf = vec![];
        LiveEvent::Realtime(SystemRealtime::Reset)
            .write_std(&mut buf)
            .unwrap();

        for output in self.outputs.iter() {
            if let Some(output) = output {
                output.send(buf.clone()).await.unwrap();
            }
        }
    }
}
