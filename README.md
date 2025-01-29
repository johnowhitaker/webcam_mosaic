# webcam_mosaic

Scanning slides with a microscope, a 3D printer and a webcam

```bash
python3 mosaic.py \
  --serial_port /dev/ttyUSB0 \
  --camera_device /dev/video4 \
  --prefix "capture/myscan" \
  --x_step 1.5 \
  --y_step 1.5 \
  --n_x 3 \
  --n_y 3 \
  --settle_time 300
```

Result: 
![results](results.png)

These can be combined with OpenCV's stitcher (see the 'combine_images' notebook) but another option is to use [this app](https://play.google.com/store/apps/details?id=com.bcdvision.mapstitch&hl=en_US) and a smartphone to manually capture images (I use a remote trigger) while the stage moves through the grid.