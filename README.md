# mipi_csi_cam_ros2

Самостоятельный ROS 2 пакет для публикации кадров с MIPI CSI камер через
GStreamer-пайплайны.

Поддерживаемые backend:

- `libcamera`: Raspberry Pi 5 через `libcamerasrc`;
- `v4l2`: платы RK3588/RK3588S через V4L2-устройства Rockchip `rkisp1`.

Пакет содержит:

- `mipi_csi_cam_ros2_node`: захватывает кадры через OpenCV GStreamer backend и
  публикует `sensor_msgs/msg/Image`;
- публикацию `camera_info` через `camera_info_manager`;
- launch-файл с namespace камеры, выбором calibration-файла и статическим TF;
- YAML-файлы калибровки для распространенных разрешений IMX219 и OV5647.

## Поддерживаемое железо

| Платформа | Сенсоры | Backend | Типичное устройство |
|---|---|---|---|
| Raspberry Pi 5 | IMX219, OV5647 | `libcamera` / `libcamerasrc` | автообнаружение libcamera |
| Radxa CM5 / Orange Pi 5 Pro | IMX219, OV5647, IMX708 | `v4l2` / `v4l2src` | `/dev/video11` |

## Зависимости

Установите ROS 2 Humble и зависимости для сборки/запуска:

```bash
sudo apt update
sudo apt install -y \
    libgstreamer1.0-dev \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libcamera \
    v4l-utils \
    ros-humble-camera-info-manager \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-tf2-ros \
    python3-yaml
```

`gstreamer1.0-libcamera` нужен только для backend `libcamera` на Raspberry Pi.
RK3588/RK3588S использует `v4l2src`.

## Структура workspace

Клонируйте репозиторий в директорию `src` ROS 2 workspace:

```bash
mkdir -p ~/mipi_csi_cam_ros2_ws/src
cd ~/mipi_csi_cam_ros2_ws/src
git clone <repo-url> mipi_csi_cam_ros2
```

## Сборка

```bash
cd ~/mipi_csi_cam_ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select mipi_csi_cam_ros2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Для сборки прямо на целевом устройстве можно включить native CPU flags:

```bash
colcon build --packages-select mipi_csi_cam_ros2 --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DMIPI_CSI_CAM_ROS2_ENABLE_NATIVE_TUNE=ON
```

Для cross-build и Docker buildx оставляйте `MIPI_CSI_CAM_ROS2_ENABLE_NATIVE_TUNE`
выключенным.

## Запуск на Raspberry Pi 5

Используйте backend `libcamera`:

```bash
ros2 launch mipi_csi_cam_ros2 camera.launch.py \
    source_type:=libcamera \
    sensor:=imx219 \
    width:=1280 \
    height:=960 \
    fps:=30 \
    camera_id:=1
```

Если подключено несколько libcamera-камер, передайте точное имя камеры:

```bash
libcamera-hello --list-cameras

ros2 launch mipi_csi_cam_ros2 camera.launch.py \
    source_type:=libcamera \
    sensor:=imx219 \
    camera_id:=0 \
    camera_name:="/base/axi/pcie@1000120000/rp1/i2c@88000/imx219@10"
```

## Запуск на RK3588/RK3588S

Используйте backend `v4l2` для Rockchip vendor kernels, где камера доступна
через V4L2-устройства `rkisp1`:

```bash
ros2 launch mipi_csi_cam_ros2 camera.launch.py \
    source_type:=v4l2 \
    sensor:=ov5647 \
    width:=640 \
    height:=480 \
    fps:=25 \
    camera_id:=1 \
    video_device:=/dev/video11 \
    format:=NV12 \
    io_mode:=dmabuf
```

Пример запуска с меньшим разрешением:

```bash
ros2 launch mipi_csi_cam_ros2 camera.launch.py \
    source_type:=v4l2 \
    sensor:=ov5647 \
    width:=320 \
    height:=240 \
    fps:=25 \
    camera_id:=1 \
    video_device:=/dev/video11
```

Если `dmabuf` недоступен в целевом ядре, используйте:

```bash
ros2 launch mipi_csi_cam_ros2 camera.launch.py \
    source_type:=v4l2 \
    io_mode:=mmap
```

V4L2 backend строит пайплайн такого вида:

```text
v4l2src device=/dev/video11 do-timestamp=true io-mode=dmabuf
  ! video/x-raw,format=NV12,width=640,height=480
  ! videoconvert
  ! video/x-raw,format=BGR
  ! appsink
```

По умолчанию V4L2 caps не содержат framerate, это лучше совпадает с поведением
многих RK3588 vendor kernels. Если устройство принимает явный framerate в caps,
передайте `v4l2_use_framerate_caps:=true`.

## Изменение размера выходного потока

Можно захватывать кадр в одном разрешении, а публиковать в другом:

```bash
ros2 launch mipi_csi_cam_ros2 camera.launch.py \
    source_type:=libcamera \
    sensor:=imx219 \
    width:=1280 \
    height:=960 \
    resize_w:=640 \
    resize_h:=480 \
    fps:=30 \
    camera_id:=1
```

## Топики

Для `camera_id:=1` launch-файл публикует:

| Топик | Тип |
|---|---|
| `/camera_1/image_raw` | `sensor_msgs/msg/Image` |
| `/camera_1/camera_info` | `sensor_msgs/msg/CameraInfo` |

QoS для изображения по умолчанию `best_effort`, чтобы медленные подписчики не
тормозили захват. Если нужен reliable image topic, используйте
`qos_reliable:=true`.

## Параметры

| Параметр | По умолчанию | Описание |
|---|---|---|
| `source_type` | `libcamera` | Backend захвата: `libcamera` или `v4l2` |
| `sensor` | `imx219` | Имя сенсора для логов и выбора calibration-файла |
| `width` | `1280` | Ширина захвата |
| `height` | `960` | Высота захвата |
| `fps` | `30` | Запрошенная частота кадров |
| `camera` | `1` | Алиас для `camera_id` |
| `camera_id` | `1` | Суффикс namespace камеры в launch-файле |
| `resize_w` | `0` | Ширина публикуемого изображения, `0` оставляет ширину захвата |
| `resize_h` | `0` | Высота публикуемого изображения, `0` оставляет высоту захвата |
| `video_device` | `/dev/video11` | V4L2-устройство для `source_type:=v4l2` |
| `format` | `NV12` | Pixel format для V4L2 caps |
| `io_mode` | `dmabuf` | V4L2 io-mode; при необходимости используйте `mmap` или `auto` |
| `v4l2_use_framerate_caps` | `false` | Добавить `framerate=<fps>/1` в V4L2 caps |
| `qos_reliable` | `false` | Использовать reliable QoS для публикации изображений |
| `camera_name` | `""` | Опциональное свойство libcamera camera-name |

Нода также принимает прямые параметры:

| Параметр ноды | Описание |
|---|---|
| `topic` | Полное имя image topic |
| `calibration_file` | Абсолютный путь к calibration YAML |
| `frame_id` | Frame id для заголовков image и camera info |

## Калибровка

Файлы калибровки лежат в `config/` и используют соглашение об именах:

```text
{sensor}_{width}x{height}.yaml
```

Пример:

```text
config/imx219_1280x960.yaml
```

Launch-файл автоматически выбирает calibration-файл по сенсору и размеру
захвата.

Запуск калибровки:

```bash
ros2 run camera_calibration cameracalibrator \
    --size 8x6 \
    --square 0.025 \
    image:=/camera_1/image_raw \
    camera:=/camera_1
```

## Static TF

Статические трансформации камер задаются в `config/params.yaml`:

```yaml
camera_tf_1:
  ros__parameters:
    parent_frame: "base_link"
    xyz: [0.065, 0.0, -0.03]
    rpy: [-1.5707963, 0.0, 3.1415926]
```

Launch-файл запускает `tf2_ros/static_transform_publisher` для выбранного
`camera_id`.

## Отладка

Проверка видимости libcamera на Raspberry Pi:

```bash
libcamera-hello --list-cameras
```

Проверка GStreamer-пайплайна на Raspberry Pi:

```bash
gst-launch-1.0 libcamerasrc \
    ! video/x-raw,width=1280,height=960,framerate=30/1 \
    ! videoconvert \
    ! autovideosink
```

Проверка V4L2 на RK3588/RK3588S:

```bash
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video11 --list-formats-ext
```

Проверка GStreamer-пайплайна на RK3588/RK3588S:

```bash
gst-launch-1.0 v4l2src device=/dev/video11 do-timestamp=true io-mode=dmabuf \
    ! video/x-raw,format=NV12,width=640,height=480 \
    ! videoconvert \
    ! autovideosink
```

Проверка ROS-топиков:

```bash
ros2 topic list | grep camera
ros2 topic hz /camera_1/image_raw
ros2 topic echo /camera_1/camera_info --once
```

Просмотр изображения:

```bash
ros2 run rqt_image_view rqt_image_view /camera_1/image_raw
```

## Заметки по репозиторию

Артефакты сборки намеренно исключены из git:

- `build/`
- `install/`
- `log/`

Репозиторий рассчитан на клонирование как обычный ROS 2 пакет внутрь директории
`src/` любого workspace.
