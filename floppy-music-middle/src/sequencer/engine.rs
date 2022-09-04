use std::{
    pin::Pin,
    sync::{atomic::AtomicUsize, Arc},
};

use midly::{
    live::{LiveEvent, SystemCommon, SystemRealtime},
    MetaMessage, MidiMessage, Timing, Track, TrackEventKind,
};
use serde::Serialize;
use tokio::{select, sync::mpsc, time::Instant};

use super::{time_controller::TimeController, watch};

/*pub struct MidiEngine<'a> {
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
}*/

#[derive(Debug, Copy, Clone, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum EngineState {
    Running,
    Paused,
    Stopping,
    Stopped,
}
impl EngineState {
    pub fn is_running(&self) -> bool {
        if let Self::Running = self {
            true
        } else {
            false
        }
    }

    pub fn is_paused(&self) -> bool {
        if let Self::Paused = self {
            true
        } else {
            false
        }
    }

    pub fn is_stopped(&self) -> bool {
        if let Self::Stopped = self {
            true
        } else {
            false
        }
    }

    pub fn is_stopping(&self) -> bool {
        if let Self::Stopping = self {
            true
        } else {
            false
        }
    }
}

pub struct MidiEngine {
    time_controller: TimeController,
    state_watch: watch::Sender<EngineState>,
    instance_count: Arc<AtomicUsize>,
}
impl MidiEngine {
    pub fn new() -> Self {
        let tempo = 0x07A120;
        let (state_watch, _) = watch::channel(EngineState::Stopped);

        Self {
            time_controller: TimeController::new(tempo, 1),
            state_watch,
            instance_count: Arc::new(AtomicUsize::new(0)),
        }
    }

    pub async fn play_to(&mut self, bytes: &[u8], output: mpsc::Sender<Vec<u8>>) {
        let bytes = bytes.to_vec();

        use std::sync::atomic::Ordering;

        if self.instance_count.load(Ordering::Relaxed) > 0 {
            self.stop().await;
        }

        let mut rx = self.state_watch.subscribe();
        let tx = self.state_watch.clone();
        let mut time_controller = self.time_controller;

        async fn until_stopping(mut rx: watch::Receiver<EngineState>) {
            while rx.changed().await.is_ok() {
                if rx.borrow().is_stopping() {
                    return;
                }
            }
        }

        let instance_count = self.instance_count.clone();

        tokio::spawn(async move {
            let bytes = bytes;

            let file = midly::Smf::parse(&bytes[..]).unwrap();

            let timing = match file.header.timing {
                Timing::Metrical(ppq) => ppq.as_int(),
                _ => unimplemented!("Only the Timing method Metrical is implemented"),
            };

            time_controller.set_ppq(timing as u32);

            let mut threads = file
                .tracks
                .into_iter()
                .map(|track| {
                    let rx = tx.subscribe();

                    MidiEngineThread::new(time_controller, output.clone(), track)
                })
                .collect::<Vec<_>>();

            let num_instances = threads.len();
            instance_count.fetch_add(num_instances, Ordering::Relaxed);

            select! {
                _ = futures::future::join_all(threads.iter_mut().map(|thread| {
                    let rx = tx.subscribe();
                    async move { thread.run(rx).await }
                })) => {},
                _ = until_stopping(rx) => {},
            }

            tx.send(EngineState::Stopping);

            futures::future::join_all(threads.into_iter().map(|thread| async move {
                thread.cleanup().await;
            }))
            .await;

            tx.send(EngineState::Stopped);

            instance_count.fetch_sub(num_instances, Ordering::Relaxed);
        });

        self.state_watch.send(EngineState::Running).unwrap();
    }

    pub fn pause(&self) -> Result<(), ()> {
        if !self.state_watch.borrow().is_running() {
            return Err(());
        }

        self.state_watch.send(EngineState::Paused);

        Ok(())
    }

    pub fn resume(&self) -> Result<(), ()> {
        if !self.state_watch.borrow().is_paused() {
            return Err(());
        }

        self.state_watch.send(EngineState::Running);

        Ok(())
    }

    pub async fn stop(&mut self) {
        self.state_watch.send(EngineState::Stopping);

        self.until_stopped().await;
    }

    pub async fn until_stopping(&self) {
        let mut rx = self.state_watch.subscribe();

        if rx.borrow().is_stopping() {
            return;
        }

        while rx.changed().await.is_ok() {
            if rx.borrow().is_stopping() {
                return;
            }
        }
    }

    pub async fn until_stopped(&self) {
        let mut rx = self.state_watch.subscribe();

        if rx.borrow().is_stopped() {
            return;
        }

        while rx.changed().await.is_ok() {
            if rx.borrow().is_stopped() {
                return;
            }
        }
    }

    pub fn get_state(&self) -> EngineState {
        *self.state_watch.borrow()
    }

    pub fn watch_state(&self) -> watch::Receiver<EngineState> {
        self.state_watch.subscribe()
    }
}

struct MidiEngineThread<'a> {
    time_controller: TimeController,
    output: mpsc::Sender<Vec<u8>>,
    track: Track<'a>,
}
impl<'a> MidiEngineThread<'a> {
    pub fn new(
        time_controller: TimeController,
        output: mpsc::Sender<Vec<u8>>,
        track: Track<'a>,
    ) -> Self {
        Self {
            time_controller,
            output,
            track,
        }
    }

    async fn try_pause(
        rx: &mut watch::Receiver<EngineState>,
        tx: &mut mpsc::Sender<Vec<u8>>,
        mut sleep: Pin<&mut tokio::time::Sleep>,
    ) {
        let left = sleep.deadline() - Instant::now();

        Self::pause(rx, tx).await;

        sleep.as_mut().reset(Instant::now() + left);

        sleep.await;
    }

    pub async fn run(&mut self, mut rx: watch::Receiver<EngineState>) {
        for event in self.track.iter() {
            let sleep = tokio::time::sleep(tokio::time::Duration::from_micros(
                self.time_controller.delta_micros(event.delta.as_int()) as u64,
            ));
            tokio::pin!(sleep);

            let cloned = tokio::time::sleep_until(sleep.deadline());
            tokio::pin!(cloned);

            select! {
                _ = sleep => {},
                Ok(_) = rx.changed(), if rx.borrow().is_paused() => Self::try_pause(&mut rx, &mut self.output, cloned).await,
            }

            let event = event.kind;
            match event.as_live_event() {
                Some(event) => {
                    let mut buf = vec![];
                    event.write_std(&mut buf).unwrap();

                    self.output.send(buf).await.unwrap();
                }
                _ => {
                    if let TrackEventKind::Meta(msg) = event {
                        match msg {
                            MetaMessage::Tempo(tempo) => {
                                self.time_controller.set_tempo(tempo.as_int())
                            }
                            _ => {}
                        }
                    } else {
                        unreachable!();
                    }
                }
            }
        }
    }

    async fn pause(rx: &mut watch::Receiver<EngineState>, tx: &mut mpsc::Sender<Vec<u8>>) {
        let mut buf = vec![];
        LiveEvent::Realtime(SystemRealtime::Stop)
            .write_std(&mut buf)
            .unwrap();

        tx.send(buf).await.unwrap();

        while rx.changed().await.is_ok() {
            if let EngineState::Running = *rx.borrow() {
                break;
            }
        }

        let mut buf = vec![];
        LiveEvent::Realtime(SystemRealtime::Start)
            .write_std(&mut buf)
            .unwrap();

        tx.send(buf).await.unwrap();
    }

    pub async fn cleanup(mut self) {
        let mut buf = vec![];
        LiveEvent::Realtime(SystemRealtime::Reset)
            .write_std(&mut buf)
            .unwrap();

        self.output.send(buf).await.unwrap();
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

    pub async fn cleanup(self) {
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
