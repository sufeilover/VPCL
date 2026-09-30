# HUIYU_PAPER 本地发布包

按论文章节与实验整理的平台代码和处理后数据。当前是本地候选发布版，尚未上传 GitHub，也未给第三方代码重新指定许可证。

先查看英文 README 的目录索引，再执行：

```powershell
python -m pip install -r requirements-analysis.txt
python tools/verify_package.py
python reproduce.py all
```

`reproduce.py` 是新建的便携离线入口，使用包内处理结果，不启动 AC、不发送控制命令、不修改原始输入。`source_scripts` 保留历史计算源码，不等于所有脚本都已经免配置可运行。

第四章：五指标、归一化、收益损失与 λ 敏感性；AIpush 独立汇总；四配置时间分析。
第五章：案例1预测视觉反馈和持续帧数；案例2 AI 与辅助驾驶全圈对比；案例3五位真人、每人两条件各三次的匿名汇总。

注意：原始大表、逐500步的全部明细、AC车辆/赛道资源、ACTI等插件安装包未打包。已补入旧VPCL仓库的模型C++源码和 `platform/bin/PyProjectD.pyd`，但不代表已具备全部构建依赖或完成运行兼容性验证。当前可以基于已处理数据重新计算部分图表，不能声称从原始采集到所有图表均已一键复现。

## 从旧仓库合并的模型文件和使用指导

- `Model/code/projectd-core-develop-src/` 对应本包 `platform/model/projectd-core-develop-src/`，共204个文件，保留原内容。
- `Model/bin/` 对应本包 `platform/bin/`，包含 `PyProjectD.pyd`。
- 下载匹配的上游 ProjectD-Core 工程，将源码覆盖包的内容放入工程的 `src/` 下，再使用 Visual Studio 2019 重编译；不要直接覆盖工程根目录。修改C++后必须重新编译。
- 旧环境使用Python 3.10；二进制需匹配Python ABI、系统架构和依赖DLL。不兼容时应在本机编译，将生成的接口放入 `platform/bin/`。
- 先合法安装AC及车辆/赛道资源、原始ACTI和控制插件。替换插件文件前备份。本包提供 `platform/ac/unbound/Unbound.lua`，不提供完整ACTI安装包。
- 运行顺序：启动AC，再运行 `platform/ac/` 下对应客户端，最后运行 `platform/model/` 下配对模型脚本。具体文件对照见 `platform/README.md`。先检查本机路径、端口、资源和接口配置。
- 离线分析仍使用 `reproduce.py`，不需要启动游戏，也不会加载模型二进制。

感谢ProjectD-Core、ACTI、Custom Shaders Patch、unbound的开发者和维护者，以及Assetto Corsa和相关第三方库的作者。原README和许可证保存在 `docs/legacy_vpcl/` 供追溯；旧文档中的路径和实验说明以本包新版README为准。此次不为第三方源码或新版数据重新指定许可证。

发布前请处理 `docs/RELEASE_CHECKLIST.md`：特别是 ProjectD-Core 使用限制、OUT几何定义，以及实际运行版本。此次合并前，案例2源码已被修改为10步；旧清单记录的4步不再描述当前文件，但历史采集版本仍需确认。此次未修改该脚本。
