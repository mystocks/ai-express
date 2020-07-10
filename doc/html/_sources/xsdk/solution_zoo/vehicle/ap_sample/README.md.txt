# 车路协同方案AP侧Sample
## 介绍
该Sample提供AP侧基于hbipc bifspi通路的接收样例。程序启动后会请求连接CP端。连接成功后开始接收数据并进行解析。

## 编译
```
bash build.sh
 ```
## 打包
 ```
bash deploy.sh
 ```
该脚本会在当前目录下创建deploy文件夹，里面包含运行时库和可执行程序。

## 运行
将部署包拷贝到板子上，即可运行。
 ```
sh run.sh
 ```

