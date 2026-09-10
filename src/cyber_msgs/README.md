# cyber_msgs

ros msgs for autodrive_ros

## 使用步骤 

1. 删除原仓库里cyber_msgs目录，并且提交到git上

2. 引入cyber_msgs作为autodrive_ros的submodule, 路径为src/cyber_msgs

```
git submodule add http://112.27.121.65:28088/system/cyber_msgs src/cyber_msgs
```

3. 根据当前编译的ros版本，拷贝package.xml文件

```
cp package_ros*.xml package.xml
```

4. 编译msg

* 交叉环境
```
bash cc_build_ros2.sh cyber_msgs
```

* orin环境
```
make msgs
```