use midly::{MetaMessage, MidiMessage, Timing, Track, TrackEventKind};
use tokio::sync::mpsc;

use super::time_controller::TimeController;

pub struct MidiEngine<'a> {
    time_controller: TimeController,
    file: midly::Smf<'a>,
}
impl<'a> MidiEngine<'a> {
    pub fn new(file: midly::Smf<'a>) -> Self {
        let tempo = file
            .tracks
            .iter()
            .map(|track| track.iter())
            .flatten()
            .find_map(|event| {
                if let TrackEventKind::Meta(MetaMessage::Tempo(tempo)) = event.kind {
                    Some(tempo)
                } else {
                    None
                }
            })
            .unwrap();

        let timing = match file.header.timing {
            Timing::Metrical(ppq) => ppq,
            _ => unimplemented!("Only the Timing method Metrical is implemented"),
        };

        Self {
            time_controller: TimeController::new(tempo.as_int(), timing.as_int() as u32),
            file,
        }
    }

    pub async fn play_to(&self, outputs: Vec<Option<mpsc::Sender<Vec<u8>>>>) {
        let threads = self
            .file
            .tracks
            .iter()
            .enumerate()
            .zip(outputs.into_iter())
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
        time_controller: TimeController,
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
                _ => {}
            }
        }
    }
}
