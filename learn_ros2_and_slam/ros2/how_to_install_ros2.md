## 使用fishros


```bash
wget http://fishros.com/install -O fishros && . fishros
```
选择安装桌面版(desktop)

### 环境配置

强烈推荐使用zsh，以减少你输入冗长命令的时间。
可以添加source /opt/ros/humble/setup.zsh这样的命令
严禁直接在.zshrc或者.bahsrc中添加source到工作区的命令！

### 一些宏

让你的ros2的终端输出包含颜色(warning的黄色和error的红色)，方便你观察节点启动的状态和是否报错
```bash
export RCUTILS_COLORIZED_OUTPUT=1
```
