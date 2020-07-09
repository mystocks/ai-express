XStream
=======

# 介绍

XStream 发版包编译、运行介绍。

# 编译
## 编译环境

需提前准备好交叉编译工具链，默认路径如下：
 ```
set(CMAKE_C_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER /opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin/aarch64-linux-gnu-g++)
 ```
 如果交叉编译工具链地址变动，需同步修改CMakeLists.txt
 ## 编译命令
执行如下命令，编译的可执行文件和库在build/bin和build/lib目录下
 ```
bash build.sh
 ```
# 部署
 ```
bash deploy.sh
 ```
该脚本会创建deploy部署包，包括如下几个部分：

| 名称             |             备注 |
| ---------------- | ---------------: |
| lib              |       动态依赖库 |
| models           |         模型集合 |
| face_solution    |     人脸解决方案 |
| body_solution    |     人体解决方案 |
| vehicle_solution | 车路协同解决方案 |
| face_body_multisource    |     多路输入多workflow解决方案 |
| ssd_test         | ssd检测模型示例程序 |
| configs          |     vio 配置文件 |
| run.sh           |         运行脚本 |

# 运行
直接运行run.sh脚本即可运行指定的测试程序。默认使用96baord配置，添加`2610`选项以在2610平台上运行。各个测试程序的介绍及运行方法请参考相应源码目录下的README.md
 ```
sh run.sh [ face | face_recog | body | xbox | behavior | vehicle | face_body_multisource | ssd_test ] [ 96board | 2610 | x3dev ] [ imx327 | os8a10 | s5kgm | s5kgm_2160p]
 ```
 ## 硬件说明
| 开发板           |             备注                            |
| --------------  | ---------------:                            |
| 96board         | X2 96board开发板，demo中只配置了1080P的sensor  |
| 2610            | X2 2610 原型机，demo中只配置了1080P的sensor    |
| x3dev           | X3 开发板，demo中适配了三种sensor，分别为imx327（1080P），os8a10（2160P），s5kgm（4000x3000）, s5kgm_2160p(2160P)|


