use std::{
    collections::HashMap,
    path::{Path, PathBuf},
    sync::Arc,
};

use notify::{Config, EventKind, RecommendedWatcher, RecursiveMode, Watcher};
use rocket::{
    fs::NamedFile, http::ContentType, response::status::NotFound, serde::json::Json, State,
};
use rocket_auth::User;
use thiserror::Error;
use tokio::{
    fs,
    runtime::Handle,
    sync::{mpsc, RwLock},
};

pub struct File {
    pub name: String,
    pub path: PathBuf,
}

pub(super) type Files = Vec<String>;
pub(super) type FileMap = HashMap<String, File>;

#[derive(Debug, Error)]
pub enum CreateFileMapError {
    #[error(transparent)]
    IoError(#[from] tokio::io::Error),
    #[error(transparent)]
    NotifyError(#[from] notify::Error),
}

pub async fn create_file_map<P: AsRef<Path>>(
    path: P,
) -> Result<Arc<RwLock<FileMap>>, CreateFileMapError> {
    let path = path.as_ref();

    let mut files = FileMap::new();

    let mut dir = fs::read_dir(path).await?;
    while let Some(entry) = dir.next_entry().await? {
        let path = entry.path();

        if path.is_dir() {
            continue;
        }

        let mut file_name = path.file_name().unwrap().to_string_lossy().to_string();

        files.insert(
            file_name.clone(),
            File {
                name: file_name,
                path: path.to_path_buf(),
            },
        );
    }

    let files = Arc::new(RwLock::new(files));

    let watcher_files = files.clone();
    let path = path.to_path_buf();

    tokio::spawn(async move {
        start_watcher(path, watcher_files).await;
    });

    Ok(files)
}

async fn start_watcher<P: AsRef<Path>>(path: P, files: Arc<RwLock<FileMap>>) -> notify::Result<()> {
    let (mut tx, mut rx) = mpsc::channel(10);

    let handle = Handle::current();
    let mut watcher = RecommendedWatcher::new(
        move |result| {
            handle.block_on(async { tx.send(result).await.unwrap() });
        },
        Config::default(),
    )?;

    watcher.watch(path.as_ref(), RecursiveMode::Recursive)?;

    while let Some(result) = rx.recv().await {
        match result {
            Ok(event) => match event.kind {
                EventKind::Create(_) => {
                    let mut files = files.write().await;
                    for path in event.paths {
                        if !path.is_file() {
                            continue;
                        }

                        let file_name = path.file_name().unwrap().to_string_lossy().to_string();

                        files.insert(
                            file_name.clone(),
                            File {
                                name: file_name,
                                path,
                            },
                        );
                    }
                }
                EventKind::Remove(_) => {
                    let mut files = files.write().await;
                    for path in event.paths {
                        files.remove(&path.file_name().unwrap().to_string_lossy().to_string());
                    }
                }
                _ => {}
            },
            Err(e) => {}
        }
    }

    Ok(())
}

#[get("/files")]
pub async fn get_files(user: User, files: &State<Arc<RwLock<FileMap>>>) -> Json<Files> {
    let mut files = files
        .read()
        .await
        .keys()
        .map(|p| p.clone())
        .collect::<Vec<_>>();

    files.sort();

    Json(files)
}

#[get("/file/<file>")]
pub async fn get_file(
    user: User,
    files: &State<Arc<RwLock<FileMap>>>,
    file: &str,
) -> Result<NamedFile, NotFound<String>> {
    let file_path = files
        .read()
        .await
        .get(&file.to_string())
        .map(|p| p.path.clone())
        .ok_or(NotFound(file.to_string()))?;

    NamedFile::open(&file_path)
        .await
        .map_err(|e| NotFound(e.to_string()))
}
