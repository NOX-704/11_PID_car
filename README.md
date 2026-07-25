# 11_PID_car

八路循迹模块+PID驱动

## VS Code 跨平台路径配置

`.vscode/c_cpp_properties.json` 不保存个人电脑的绝对路径。每台电脑只需配置以下两个用户环境变量：

- `MSPM0_SDK_ROOT`：MSPM0 SDK 安装目录
- `TI_ARMCLANG_ROOT`：TI ArmClang 版本目录

macOS 示例：

```bash
export MSPM0_SDK_ROOT=/Users/rylum/ti/mspm0_sdk_2_10_00_04
export TI_ARMCLANG_ROOT=/Applications/ti/ccs2100/ccs/tools/compiler/ti-cgt-armllvm_5.1.1.LTS
```

Windows PowerShell 示例（路径按实际安装位置修改）：

```powershell
[Environment]::SetEnvironmentVariable("MSPM0_SDK_ROOT", "D:\ti\ccs2050\mspm0_sdk_2_10_00_04", "User")
[Environment]::SetEnvironmentVariable("TI_ARMCLANG_ROOT", "D:\ti\ccs2050\ccs\tools\compiler\ti-cgt-armllvm_4.0.4.LTS", "User")
```

配置后重新启动 VS Code，并通过 `C/C++: Select IntelliSense Configuration` 选择当前操作系统对应的配置。
