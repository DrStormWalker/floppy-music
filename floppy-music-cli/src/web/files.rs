use std::{
    collections::HashMap,
    path::{Path, PathBuf},
    sync::Arc,
};

use notify::{Config, EventKind, RecommendedWatcher, RecursiveMode, Watcher};
use rocket::{
    fs::NamedFile,
    http::ContentType,
    response::status::NotFound,
    serde::{json::Json, Deserialize, Serialize},
    State,
};
use rocket_auth::User;
use thiserror::Error;
use tokio::{
    fs, io,
    runtime::Handle,
    sync::{mpsc, RwLock},
};

#[derive(Serialize, Deserialize)]
pub struct File {
    pub name: String,
    pub path: PathBuf,
}

pub(super) type Files = Vec<String>;

#[derive(Debug, Error)]
pub enum FileMapError {
    #[error(transparent)]
    IoError(#[from] tokio::io::Error),
    #[error(transparent)]
    TomlError(#[from] toml::de::Error),
}

pub struct FileMap {
    file_path: PathBuf,
    dir_path: PathBuf,
    pub map: Arc<RwLock<HashMap<String, File>>>,
}
impl FileMap {
    pub fn new(file_path: impl AsRef<Path>, dir_path: impl AsRef<Path>) -> Self {
        Self {
            map: Arc::new(RwLock::new(HashMap::new())),
            file_path: file_path.as_ref().to_path_buf(),
            dir_path: dir_path.as_ref().to_path_buf(),
        }
    }

    pub async fn load_or_create_file(
        file_path: impl AsRef<Path>,
        dir_path: impl AsRef<Path>,
    ) -> Result<Self, FileMapError> {
        let file = fs::OpenOptions::new()
            .create(true)
            .read(true)
            .write(true)
            .open(file_path.as_ref())
            .await?;

        let mut stream = io::BufStream::new(file);

        let mut contents = String::new();

        use io::AsyncReadExt as _;
        stream.read_to_string(&mut contents).await?;

        let map = if let Ok(map) = toml::from_str(&contents) {
            map
        } else {
            HashMap::new()
        };

        let contents = toml::to_string_pretty(&map).unwrap();

        use io::AsyncWriteExt as _;
        stream.write_buf(&mut contents.as_bytes());

        Ok(Self {
            file_path: file_path.as_ref().to_path_buf(),
            dir_path: dir_path.as_ref().to_path_buf(),
            map: Arc::new(RwLock::new(map)),
        })
    }

    pub async fn refresh_from_file(&mut self) -> Result<(), FileMapError> {
        let file = fs::OpenOptions::new()
            .read(true)
            .open(&self.file_path)
            .await?;

        let mut stream = io::BufStream::new(file);

        let mut contents = String::new();
        use io::AsyncReadExt as _;
        stream.read_to_string(&mut contents).await?;

        let map = toml::from_str(&contents)?;

        *self.map.write().await = map;

        Ok(())
    }

    pub async fn refresh_to_file(&mut self) -> Result<(), FileMapError> {
        let file = rocket::tokio::fs::OpenOptions::new()
            .write(true)
            .open(&self.file_path)
            .await?;

        let mut stream = rocket::tokio::io::BufStream::new(file);

        let contents = toml::to_string_pretty(&*self.map.read().await).unwrap();

        use io::AsyncWriteExt as _;
        stream.write_all_buf(&mut contents.as_bytes()).await?;

        Ok(())
    }

    pub async fn refresh_from_dir(&mut self) -> Result<(), FileMapError> {
        use async_walkdir::WalkDir;
        use rocket::futures::StreamExt as _;
        let mut dir = WalkDir::new(&self.dir_path);
        while let Some(entry) = dir.next().await {
            let entry = entry?;

            let path = entry.path();
            let id = path
                .strip_prefix(&self.dir_path)
                .unwrap()
                .to_string_lossy()
                .to_string();

            if self.map.read().await.contains_key(&id) {
                continue;
            }

            let file = File {
                name: entry.file_name().to_string_lossy().to_string(),
                path: entry.path(),
            };

            self.map.write().await.insert(id, file);
        }

        Ok(())
    }

    pub async fn register_dir_watcher(&self) -> notify::Result<()> {
        let (tx, mut rx) = mpsc::channel(10);

        let handle = Handle::current();
        let mut watcher = RecommendedWatcher::new(
            move |result| {
                handle.block_on(async { tx.send(result).await.unwrap() });
            },
            Config::default(),
        )?;

        watcher.watch(self.dir_path.as_ref(), RecursiveMode::Recursive)?;

        let files = self.map.clone();
        let dir_path = self.dir_path.clone();
        let file_path = self.file_path.clone();
        tokio::spawn(async move {
            while let Some(result) = rx.recv().await {
                match result {
                    Ok(event) => match event.kind {
                        EventKind::Create(_) => {
                            let mut files = files.write().await;
                            for path in event.paths {
                                if !path.is_file() {
                                    continue;
                                }

                                let file_name =
                                    path.file_name().unwrap().to_string_lossy().to_string();

                                if files.contains_key(&file_name) {
                                    continue;
                                }

                                let id = path
                                    .strip_prefix(&dir_path)
                                    .unwrap()
                                    .to_string_lossy()
                                    .to_string();

                                files.insert(
                                    id,
                                    File {
                                        name: file_name,
                                        path,
                                    },
                                );

                                let file = fs::OpenOptions::new()
                                    .write(true)
                                    .open(&file_path)
                                    .await
                                    .unwrap();

                                let mut stream = io::BufStream::new(file);

                                let contents = toml::to_string_pretty(&*files).unwrap();

                                use io::AsyncWriteExt as _;
                                stream.write_all_buf(&mut contents.as_bytes()).await;
                            }
                        }
                        EventKind::Remove(_) => {
                            let mut files = files.write().await;
                            for path in event.paths {
                                files.remove(&path.to_string_lossy().to_string());
                            }
                        }
                        _ => {}
                    },
                    Err(e) => {}
                }
            }
        });

        Ok(())
    }

    pub async fn load_from_file(
        file_path: impl AsRef<Path>,
        dir_path: impl AsRef<Path>,
    ) -> Result<Self, FileMapError> {
        let file = rocket::tokio::fs::read_to_string(file_path.as_ref()).await?;

        let map = toml::from_str(&file)?;

        Ok(Self {
            file_path: file_path.as_ref().to_path_buf(),
            dir_path: dir_path.as_ref().to_path_buf(),
            map: Arc::new(RwLock::new(map)),
        })
    }
}

/*#[derive(Debug, Error)]
pub enum CreateFileMapError {
    #[error(transparent)]
    IoError(#[from] tokio::io::Error),
    #[error(transparent)]
    NotifyError(#[from] notify::Error),
}

pub async fn create_file_map<P: AsRef<Path>, M: AsRef<Path>>(
    path: P,
    map_db: M,
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
}*/

#[get("/files")]
pub async fn get_files(user: User, files: &State<Arc<RwLock<FileMap>>>) -> Json<Files> {
    let mut files = files
        .read()
        .await
        .map
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
        .map
        .read()
        .await
        .get(&file.to_string())
        .map(|p| p.path.clone())
        .ok_or(NotFound(file.to_string()))?;

    NamedFile::open(&file_path)
        .await
        .map_err(|e| NotFound(e.to_string()))
}
