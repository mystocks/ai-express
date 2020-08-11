#!/bin/sh

usage(){
   #echo "usage: sh run.sh [ face | face_recog | body | xbox | behavior | vehicle | face_body_multisource | ssd_test ] [ 96board | 2610 | x3dev ] [ imx327 | os8a10 | s5kgm | s5kgm_2160p]"
   echo "usage: sh run.sh [ face | face_recog | body | xbox | behavior | video_box | guesture | face_body_multisource | ssd_test ] [ 96board | 2610 | x3dev | x3svb ] [ imx327 | os8a10 | s5kgm(only work for body and behavior) | s5kgm_2160p | hg | usb_cam ] [ cache | jpg | nv12 ]"
   exit 1
}

if [ $# -lt 1 ]
then
   echo "error! no test choose"
   usage
fi
chmod +x start_nginx.sh
sh start_nginx.sh

export LD_LIBRARY_PATH=./lib/
echo 1 >/sys/class/vps/mipi_host1/param/stop_check_instart
vio_cfg_file=./configs/vio_config.json.96board


if [ $# -ge 2 ]
then
  platform=$2
  sensor=$3
  source=$4
  if [ $platform == "96board" ]
  then
    sed -i 's#\("layer": \).*#\1'4',#g' configs/visualplugin_face.json
    sed -i 's#\("layer": \).*#\1'4',#g' configs/visualplugin_body.json
    sed -i 's#\("layer": \).*#\1'4',#g' configs/visualplugin_vehicle.json
    sed -i 's#\("image_width": \).*#\1'960',#g' configs/visualplugin_face.json
    sed -i 's#\("image_width": \).*#\1'960',#g' configs/visualplugin_body.json
    sed -i 's#\("image_width": \).*#\1'960',#g' configs/visualplugin_vehicle.json
    sed -i 's#\("image_height": \).*#\1'540',#g' configs/visualplugin_face.json
    sed -i 's#\("image_height": \).*#\1'540',#g' configs/visualplugin_body.json
    sed -i 's#\("image_height": \).*#\1'540',#g' configs/visualplugin_vehicle.json
elif [ $platform == "2610" ]
  then
    vio_cfg_file=./configs/vio_config.json.2610
    sed -i 's#\("layer": \).*#\1'4',#g' configs/visualplugin_face.json
    sed -i 's#\("layer": \).*#\1'4',#g' configs/visualplugin_body.json
    sed -i 's#\("layer": \).*#\1'4',#g' configs/visualplugin_vehicle.json
    sed -i 's#\("image_width": \).*#\1'960',#g' configs/visualplugin_face.json
    sed -i 's#\("image_width": \).*#\1'960',#g' configs/visualplugin_body.json
    sed -i 's#\("image_width": \).*#\1'960',#g' configs/visualplugin_vehicle.json
    sed -i 's#\("image_height": \).*#\1'540',#g' configs/visualplugin_face.json
    sed -i 's#\("image_height": \).*#\1'540',#g' configs/visualplugin_body.json
    sed -i 's#\("image_height": \).*#\1'540',#g' configs/visualplugin_vehicle.json
  elif [ $platform == "x3dev" -o $platform == "x3svb" ]
  then
    vio_cfg_file=./configs/vio_config.json.x3dev
    if [ $sensor == "imx327" ]; then
        echo "sensor is imx327, default sensor output resolution 1080P, 1080P X3 JPEG Codec..."
        sed -i 's#\("mono_vio_cfg_file": \).*#\1"configs/vio/x3dev/iot_vio_x3_imx327.json",#g' configs/vio/panel_camera.json.x3dev
        sed -i 's#\("layer": \).*#\1'0',#g' configs/visualplugin_face.json
        sed -i 's#\("layer": \).*#\1'0',#g' configs/visualplugin_body.json
        sed -i 's#\("layer": \).*#\1'0',#g' configs/visualplugin_vehicle.json
        sed -i 's#\("image_width": \).*#\1'1920',#g' configs/visualplugin_face.json
        sed -i 's#\("image_width": \).*#\1'1920',#g' configs/visualplugin_body.json
        sed -i 's#\("image_width": \).*#\1'1920',#g' configs/visualplugin_vehicle.json
        sed -i 's#\("image_height": \).*#\1'1080',#g' configs/visualplugin_face.json
        sed -i 's#\("image_height": \).*#\1'1080',#g' configs/visualplugin_body.json
        sed -i 's#\("image_height": \).*#\1'1080',#g' configs/visualplugin_vehicle.json
        sed -i 's#\("2160p_layer": \).*#\1'-1',#g' configs/visualplugin_face.json
        sed -i 's#\("1080p_layer": \).*#\1'0',#g' configs/visualplugin_face.json
        sed -i 's#\("720p_layer": \).*#\1'1',#g' configs/visualplugin_face.json
        sed -i 's#\("2160p_layer": \).*#\1'-1',#g' configs/visualplugin_body.json
        sed -i 's#\("1080p_layer": \).*#\1'0',#g' configs/visualplugin_body.json
        sed -i 's#\("720p_layer": \).*#\1'1',#g' configs/visualplugin_body.json
        sed -i 's#\("2160p_layer": \).*#\1'-1',#g' configs/visualplugin_vehicle.json
        sed -i 's#\("1080p_layer": \).*#\1'0',#g' configs/visualplugin_vehicle.json
        sed -i 's#\("720p_layer": \).*#\1'1',#g' configs/visualplugin_vehicle.json
        cp -rf body_solution/configs/box_filter_config_2M.json  body_solution/configs/box_filter_config.json
        cp -rf body_solution/configs/multitask_config_2M.json  body_solution/configs/multitask_config.json
        cp -rf body_solution/configs/guesture_multitask_2M.json  body_solution/configs/guesture_multitask.json
        cp -rf body_solution/configs/segmentation_multitask_2M.json  body_solution/configs/segmentation_multitask.json
        cp -rf face_solution/configs/face_pose_lmk_2M.json face_solution/configs/face_pose_lmk.json
#        cp -rf vehicle_solution/configs/vehicle_multitask_2M.json vehicle_solution/configs/vehicle_multitask.json
#        cp -rf vehicle_solution/configs/config_match_2M.json vehicle_solution/configs/config_match.json
    elif [ $sensor == "os8a10" ]; then
        echo "sensor is os8a10, default resolution 8M, 1080P X3 JPEG Codec..."
        echo start > /sys/devices/virtual/graphics/iar_cdev/iar_test_attr
        service adbd stop
        /etc/init.d/usb-gadget.sh start uvc-hid
        echo 0 > /proc/sys/kernel/printk
        echo 1 > /sys/class/vps/mipi_host1/param/stop_check_instart
        echo 0xc0020000 > /sys/bus/platform/drivers/ddr_monitor/axibus_ctrl/all
        echo 0x03120000 > /sys/bus/platform/drivers/ddr_monitor/read_qos_ctrl/all
        echo 0x03120000 > /sys/bus/platform/drivers/ddr_monitor/write_qos_ctrl/all
        sed -i 's#\("mono_vio_cfg_file": \).*#\1"configs/vio/x3dev/iot_vio_x3_os8a10.json",#g' configs/vio/panel_camera.json.x3dev
        if [ $platform == "x3dev" ]; then
            # set i2c5 bus
            sed -i 's#\("i2c_bus": \).*#\1'5',#g' configs/vio/x3dev/iot_vio_x3_os8a10.json
        elif [ $platform == "x3svb" ]; then
            # enable mclk output to os8a10 sensor in x3svb
            echo 1 > /sys/class/vps/mipi_host1/param/snrclk_en
            # set i2c2 bus
            sed -i 's#\("i2c_bus": \).*#\1'2',#g' configs/vio/x3dev/iot_vio_x3_os8a10.json
            # reset os8a10 mipi cam
            echo 111 > /sys/class/gpio/export
            echo out > /sys/class/gpio/gpio111/direction
            echo 0 > /sys/class/gpio/gpio111/value
            sleep 0.2
            echo 1 > /sys/class/gpio/gpio111/value
        fi
        sed -i 's#\("layer": \).*#\1'4',#g' configs/visualplugin_face.json
        sed -i 's#\("layer": \).*#\1'4',#g' configs/visualplugin_body.json
        sed -i 's#\("layer": \).*#\1'4',#g' configs/visualplugin_vehicle.json
        sed -i 's#\("image_width": \).*#\1'1920',#g' configs/visualplugin_face.json
        sed -i 's#\("image_width": \).*#\1'1920',#g' configs/visualplugin_body.json
        sed -i 's#\("image_width": \).*#\1'1920',#g' configs/visualplugin_vehicle.json
        sed -i 's#\("image_height": \).*#\1'1080',#g' configs/visualplugin_face.json
        sed -i 's#\("image_height": \).*#\1'1080',#g' configs/visualplugin_body.json
        sed -i 's#\("image_height": \).*#\1'1080',#g' configs/visualplugin_vehicle.json
        sed -i 's#\("2160p_layer": \).*#\1'0',#g' configs/visualplugin_face.json
        sed -i 's#\("1080p_layer": \).*#\1'4',#g' configs/visualplugin_face.json
        sed -i 's#\("720p_layer": \).*#\1'5',#g' configs/visualplugin_face.json
        sed -i 's#\("2160p_layer": \).*#\1'0',#g' configs/visualplugin_body.json
        sed -i 's#\("1080p_layer": \).*#\1'4',#g' configs/visualplugin_body.json
        sed -i 's#\("720p_layer": \).*#\1'5',#g' configs/visualplugin_body.json
        sed -i 's#\("2160p_layer": \).*#\1'0',#g' configs/visualplugin_vehicle.json
        sed -i 's#\("1080p_layer": \).*#\1'4',#g' configs/visualplugin_vehicle.json
        sed -i 's#\("720p_layer": \).*#\1'5',#g' configs/visualplugin_vehicle.json
        cp -rf body_solution/configs/box_filter_config_8M.json  body_solution/configs/box_filter_config.json
        cp -rf body_solution/configs/multitask_config_8M.json  body_solution/configs/multitask_config.json
        cp -rf body_solution/configs/guesture_multitask_8M.json  body_solution/configs/guesture_multitask.json
        cp -rf body_solution/configs/segmentation_multitask_8M.json  body_solution/configs/segmentation_multitask.json
        cp -rf face_solution/configs/face_pose_lmk_8M.json face_solution/configs/face_pose_lmk.json
#        cp -rf vehicle_solution/configs/vehicle_multitask_8M.json vehicle_solution/configs/vehicle_multitask.json
#        cp -rf vehicle_solution/configs/config_match_8M.json vehicle_solution/configs/config_match.json
    elif [ $sensor == "s5kgm" ]; then
        echo "sensor is s5kgm1sp, default resolution 12M, 1024*768 X3 JPEG Codec..."
        sed -i 's#\("mono_vio_cfg_file": \).*#\1"configs/vio/x3dev/iot_vio_x3_s5kgm1sp.json",#g' configs/vio/panel_camera.json.x3dev
        sed -i 's#\("layer": \).*#\1'5',#g' configs/visualplugin_face.json
        sed -i 's#\("layer": \).*#\1'5',#g' configs/visualplugin_body.json
        sed -i 's#\("layer": \).*#\1'5',#g' configs/visualplugin_vehicle.json
        sed -i 's#\("image_width": \).*#\1'1024',#g' configs/visualplugin_face.json
        sed -i 's#\("image_width": \).*#\1'1024',#g' configs/visualplugin_body.json
        sed -i 's#\("image_width": \).*#\1'1024',#g' configs/visualplugin_vehicle.json
        sed -i 's#\("image_height": \).*#\1'768',#g' configs/visualplugin_face.json
        sed -i 's#\("image_height": \).*#\1'768',#g' configs/visualplugin_body.json
        sed -i 's#\("image_height": \).*#\1'768',#g' configs/visualplugin_vehicle.json
        cp -rf body_solution/configs/box_filter_config_12M.json  body_solution/configs/box_filter_config.json
        cp -rf body_solution/configs/multitask_config_12M.json  body_solution/configs/multitask_config.json
        cp -rf body_solution/configs/guesture_multitask_12M.json  body_solution/configs/guesture_multitask.json
        cp -rf body_solution/configs/segmentation_multitask_12M.json  body_solution/configs/segmentation_multitask.json
    elif [ $sensor == "s5kgm_2160p" ]; then
        echo "sensor is s5kgm1sp, default resolution 8M, 1080P X3 JPEG Codec..."
        echo start > /sys/devices/virtual/graphics/iar_cdev/iar_test_attr
        service adbd stop
        /etc/init.d/usb-gadget.sh start uvc-hid
        echo 0 > /proc/sys/kernel/printk
        echo 1 > /sys/class/vps/mipi_host1/param/stop_check_instart
        echo 0xc0020000 > /sys/bus/platform/drivers/ddr_monitor/axibus_ctrl/all
        echo 0x03120000 > /sys/bus/platform/drivers/ddr_monitor/read_qos_ctrl/all
        echo 0x03120000 > /sys/bus/platform/drivers/ddr_monitor/write_qos_ctrl/all
        sed -i 's#\("mono_vio_cfg_file": \).*#\1"configs/vio/x3dev/iot_vio_x3_s5kgm1sp_2160p.json",#g' configs/vio/panel_camera.json.x3dev
        sed -i 's#\("layer": \).*#\1'4',#g' configs/visualplugin_face.json
        sed -i 's#\("layer": \).*#\1'4',#g' configs/visualplugin_body.json
        sed -i 's#\("layer": \).*#\1'4',#g' configs/visualplugin_vehicle.json
        sed -i 's#\("image_width": \).*#\1'1920',#g' configs/visualplugin_face.json
        sed -i 's#\("image_width": \).*#\1'1920',#g' configs/visualplugin_body.json
        sed -i 's#\("image_width": \).*#\1'1920',#g' configs/visualplugin_vehicle.json
        sed -i 's#\("image_height": \).*#\1'1080',#g' configs/visualplugin_face.json
        sed -i 's#\("image_height": \).*#\1'1080',#g' configs/visualplugin_body.json
        sed -i 's#\("image_height": \).*#\1'1080',#g' configs/visualplugin_vehicle.json
        sed -i 's#\("2160p_layer": \).*#\1'0',#g' configs/visualplugin_face.json
        sed -i 's#\("1080p_layer": \).*#\1'4',#g' configs/visualplugin_face.json
        sed -i 's#\("720p_layer": \).*#\1'5',#g' configs/visualplugin_face.json
        sed -i 's#\("2160p_layer": \).*#\1'0',#g' configs/visualplugin_body.json
        sed -i 's#\("1080p_layer": \).*#\1'4',#g' configs/visualplugin_body.json
        sed -i 's#\("720p_layer": \).*#\1'5',#g' configs/visualplugin_body.json
        sed -i 's#\("2160p_layer": \).*#\1'0',#g' configs/visualplugin_vehicle.json
        sed -i 's#\("1080p_layer": \).*#\1'4',#g' configs/visualplugin_vehicle.json
        sed -i 's#\("720p_layer": \).*#\1'5',#g' configs/visualplugin_vehicle.json
        cp -rf body_solution/configs/box_filter_config_8M.json  body_solution/configs/box_filter_config.json
        cp -rf body_solution/configs/multitask_config_8M.json  body_solution/configs/multitask_config.json
        cp -rf body_solution/configs/guesture_multitask_8M.json  body_solution/configs/guesture_multitask.json
        cp -rf body_solution/configs/segmentation_multitask_8M.json  body_solution/configs/segmentation_multitask.json
        cp -rf face_solution/configs/face_pose_lmk_8M.json face_solution/configs/face_pose_lmk.json
#        cp -rf vehicle_solution/configs/vehicle_multitask_8M.json vehicle_solution/configs/vehicle_multitask.json
#        cp -rf vehicle_solution/configs/config_match_8M.json vehicle_solution/configs/config_match.json
    elif [ $sensor == "hg" ]; then
        echo "feedback start, default resolution 1080P, 1080P X3 JPEG Codec..."
        service adbd stop
        /etc/init.d/usb-gadget.sh start uvc-hid
        vio_cfg_file=${vio_cfg_file}.hg
        cp -rf body_solution/configs/box_filter_config_2M.json  body_solution/configs/box_filter_config.json
        cp -rf body_solution/configs/multitask_config_2M.json  body_solution/configs/multitask_config.json
        cp -rf body_solution/configs/guesture_multitask_2M.json  body_solution/configs/guesture_multitask.json
        cp -rf body_solution/configs/segmentation_multitask_2M.json  body_solution/configs/segmentation_multitask.json
        cp -rf face_solution/configs/face_pose_lmk_2M.json face_solution/configs/face_pose_lmk.json
        sed -i 's#\("layer": \).*#\1'0',#g' configs/visualplugin_face.json
        sed -i 's#\("layer": \).*#\1'0',#g' configs/visualplugin_body.json
        sed -i 's#\("layer": \).*#\1'0',#g' configs/visualplugin_vehicle.json
        sed -i 's#\("image_width": \).*#\1'1920',#g' configs/visualplugin_face.json
        sed -i 's#\("image_width": \).*#\1'1920',#g' configs/visualplugin_body.json
        sed -i 's#\("image_width": \).*#\1'1920',#g' configs/visualplugin_vehicle.json
        sed -i 's#\("image_height": \).*#\1'1080',#g' configs/visualplugin_face.json
        sed -i 's#\("image_height": \).*#\1'1080',#g' configs/visualplugin_body.json
        sed -i 's#\("image_height": \).*#\1'1080',#g' configs/visualplugin_vehicle.json
        sed -i 's#\("2160p_layer": \).*#\1'-1',#g' configs/visualplugin_face.json
        sed -i 's#\("1080p_layer": \).*#\1'0',#g' configs/visualplugin_face.json
        sed -i 's#\("720p_layer": \).*#\1'1',#g' configs/visualplugin_face.json
        sed -i 's#\("2160p_layer": \).*#\1'-1',#g' configs/visualplugin_body.json
        sed -i 's#\("1080p_layer": \).*#\1'0',#g' configs/visualplugin_body.json
        sed -i 's#\("720p_layer": \).*#\1'1',#g' configs/visualplugin_body.json
        sed -i 's#\("2160p_layer": \).*#\1'-1',#g' configs/visualplugin_vehicle.json
        sed -i 's#\("1080p_layer": \).*#\1'0',#g' configs/visualplugin_vehicle.json
        sed -i 's#\("720p_layer": \).*#\1'1',#g' configs/visualplugin_vehicle.json
        if [ "$source" == "cache" ]; then
            echo "data source is cached image list"
            sed -i 's#\("data_source": \).*#\1"cached_image_list",#g' configs/vio_config.json.x3dev.hg
        elif [ "$source" == "jpg" ]; then
            echo "data source is jpg image list"
            sed -i 's#\("data_source": \).*#\1"jpeg_image_list",#g' configs/vio_config.json.x3dev.hg
            sed -i 's#\("file_path": \).*#\1"configs/vio_hg/name.list",#g' configs/vio_config.json.x3dev.hg
        elif [ "$source" == "nv12" ]; then
            echo "data source is nv12 image list"
            sed -i 's#\("data_source": \).*#\1"nv12_image_list",#g' configs/vio_config.json.x3dev.hg
            sed -i 's#\("file_path": \).*#\1"configs/vio_hg/name_nv12.list",#g' configs/vio_config.json.x3dev.hg
        else
            echo "default data source is cached image list"
            sed -i 's#\("data_source": \).*#\1"cached_image_list",#g' configs/vio_config.json.x3dev.hg
        fi
    elif [ $sensor == "usb_cam" ]; then
        echo "usb_cam feedback start, default resolution 1080P..."
        service adbd stop
        vio_cfg_file=${vio_cfg_file}.hg
        cp -rf body_solution/configs/box_filter_config_2M.json  body_solution/configs/box_filter_config.json
        cp -rf body_solution/configs/multitask_config_2M.json  body_solution/configs/multitask_config.json
        cp -rf body_solution/configs/guesture_multitask_2M.json  body_solution/configs/guesture_multitask.json
        cp -rf body_solution/configs/segmentation_multitask_2M.json  body_solution/configs/segmentation_multitask.json
        cp -rf face_solution/configs/face_pose_lmk_2M.json face_solution/configs/face_pose_lmk.json
        sed -i 's#\("layer": \).*#\1'0',#g' configs/visualplugin_face.json
        sed -i 's#\("layer": \).*#\1'0',#g' configs/visualplugin_body.json
        sed -i 's#\("layer": \).*#\1'0',#g' configs/visualplugin_vehicle.json
        sed -i 's#\("image_width": \).*#\1'1920',#g' configs/visualplugin_face.json
        sed -i 's#\("image_width": \).*#\1'1920',#g' configs/visualplugin_body.json
        sed -i 's#\("image_width": \).*#\1'1920',#g' configs/visualplugin_vehicle.json
        sed -i 's#\("image_height": \).*#\1'1080',#g' configs/visualplugin_face.json
        sed -i 's#\("image_height": \).*#\1'1080',#g' configs/visualplugin_body.json
        sed -i 's#\("image_height": \).*#\1'1080',#g' configs/visualplugin_vehicle.json
        echo "data source is usb_cam"
        sed -i 's#\("data_source": \).*#\1"usb_cam",#g' configs/vio_config.json.x3dev.hg
    elif [ $1 == "video_box" ]; then  
        cp -rf body_solution/configs/box_filter_config_2M.json  body_solution/configs/box_filter_config.json
        cp -rf body_solution/configs/multitask_config_2M.json  body_solution/configs/multitask_config.json
        cp -rf body_solution/configs/guesture_multitask_2M.json  body_solution/configs/guesture_multitask.json
        cp -rf body_solution/configs/segmentation_multitask_2M.json  body_solution/configs/segmentation_multitask.json
        cp -rf face_solution/configs/face_pose_lmk_2M.json face_solution/configs/face_pose_lmk.json
        sed -i 's#\("layer": \).*#\1'0',#g' configs/visualplugin_face.json
        sed -i 's#\("layer": \).*#\1'0',#g' configs/visualplugin_body.json
        sed -i 's#\("layer": \).*#\1'0',#g' configs/visualplugin_vehicle.json
        sed -i 's#\("image_width": \).*#\1'1920',#g' configs/visualplugin_face.json
        sed -i 's#\("image_width": \).*#\1'1920',#g' configs/visualplugin_body.json
        sed -i 's#\("image_width": \).*#\1'1920',#g' configs/visualplugin_vehicle.json
        sed -i 's#\("image_height": \).*#\1'1080',#g' configs/visualplugin_face.json
        sed -i 's#\("image_height": \).*#\1'1080',#g' configs/visualplugin_body.json
        sed -i 's#\("image_height": \).*#\1'1080',#g' configs/visualplugin_vehicle.json
    else
        echo "error! sensor" $sensor "is not supported"
    fi
  else
    echo "error! platform" $param "is not supported"
    usage
  fi
fi

for param in $1
do
  if [ $param == "face" ]
  then
    ./face_solution/face_solution $vio_cfg_file ./face_solution/configs/face_solution.json ./configs/visualplugin_face.json -i normal
  elif [ $param == "face_recog" ]
  then
    ./face_solution/face_solution $vio_cfg_file ./face_solution/configs/face_recog_solution.json ./configs/visualplugin_face.json -i normal
  elif [ $param == "body" ]
  then
    ./body_solution/body_solution $vio_cfg_file ./body_solution/configs/body_solution.json ./configs/visualplugin_body.json -i normal
  elif [ $param == "xbox" ]
  then
    ./body_solution/body_solution $vio_cfg_file ./body_solution/configs/xbox_solution.json ./configs/visualplugin_body.json -i normal
  elif [ $param == "behavior" ]
  then
    ./body_solution/body_solution $vio_cfg_file ./body_solution/configs/behavior_solution.json ./configs/visualplugin_body.json -i normal
  elif [ $param == "guesture" ]
  then
    ./body_solution/body_solution $vio_cfg_file ./body_solution/configs/guesture_solution.json ./configs/visualplugin_body.json -i normal
  elif [ $param == "video_box" ]
  then
    ./video_box/video_box $vio_cfg_file ./video_box/configs/body_solution.json  ./configs/visualplugin_body.json -i normal
#  elif [ $param == "vehicle" ]
#  then
#    ./vehicle_solution/vehicle_solution $vio_cfg_file ./vehicle_solution/configs/vehicle_solution.json ./configs/visualplugin_vehicle.json -i normal
  elif [ $param == "face_body_multisource" ]
  then
    face_body_multisource_vio=${vio_cfg_file}.hg
    if [ $platform == "x3dev" ]
    then
      sed -i 's#\("data_source": \).*#\1"cached_image_list",#g' configs/vio_config.json.x3dev.hg
      face_body_multisource_vio=configs/vio_config.json.x3dev.hg
    fi
    ./face_body_multisource/face_body_multisource ${face_body_multisource_vio} ./face_body_multisource/configs/face_body_solution.json -i normal
  elif [ $param == "ssd_test" ]
  then
    export LD_LIBRARY_PATH=./lib/:../lib
    cp configs/vio/vio_onsemi0230_fb.json  ./ssd_test/config/vio_config/vio_onsemi0230_fb.json
    cd ./ssd_test
    ./ssd_method_test
  else
    echo "error! test" $param " is not supported"
    usage
  fi
done
