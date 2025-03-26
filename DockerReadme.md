# NDT (Normal Distributions Transform) Demo with PCL in Docker

This guide provides step-by-step instructions to run the NDT point cloud registration demo using **PCL (Point Cloud Library)** inside a **Docker container** on **Ubuntu 22.04**.

---

## Prerequisites
- **Ubuntu 22.04**
- **Docker installed** ([Installation Guide](https://docs.docker.com/engine/install/ubuntu/))
- **X11 server running** (for GUI visualization)

---

## Step 1: Set Up Docker Environment

### 1.1 Allow Docker GUI Access
```bash
xhost +local:docker
```

### 1.2 Pull PCL Docker Image
```bash
docker pull pointcloudlibrary/env:22.04
```

---

## Step 2: Prepare Workspace

### 2.1 Clone PCL Tutorial Code
```bash
cd workspace/
git clone git@github.com:PointCloudLibrary/pcl.git
```


## Step 3: Run Docker Container with GUI Support and install pcl

```bash
xhost +local:docker

docker run -it \
  --user root \
  -v $(pwd)/pcl:/home \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME/.Xauthority:/home/.Xauthority:rw \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  --net=host \
  --rm \
  pointcloudlibrary/env:22.04 \
  bash
```
### 3.1 Configure Build
```bash
cd /home
mkdir build && cd build
cmake ..
```

### 3.2 Compile with 4 CPU Cores
```bash
make -j4
```

### 3.3 Install PCL
```bash
make -j4 install
```
---

## Step 4: Build and Run NDT Demo Inside the Container

### 4.1 Configure Build
```bash
cd /home
mkdir pcl_ndt_demo && cd pcl_ndt_demo
touch CMakeLists.txt main.cpp 
mkdir build && cd build
cmake ..
```
### 4.2 Compile with 4 CPU Cores
```bash
make -j4
```

### 4.3 Run NDT Registration
```bash
make -j4 install
wget https://raw.github.com/PointCloudLibrary/data/master/tutorials/room_scan1.pcd
wget https://raw.github.com/PointCloudLibrary/data/master/tutorials/room_scan2.pcd
./ndt_demo
```

---

## Step 5: Verify Results
- The visualization window should display:
  - **Red**: Original target cloud (`room_scan1.pcd`)
  - **Green**: Aligned input cloud (`room_scan2_transformed.pcd`)
- Check terminal output for convergence status and fitness score
- Transformed cloud saved as `room_scan2_transformed.pcd`

---

