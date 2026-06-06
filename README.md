# Successive Linear Approximation OPF

基于论文 *"Optimal Reactive Power Dispatch With Accurately Modeled Discrete Control Devices: A Successive Linear Approximation Approach"* 的MATLAB实现

## 项目概述

本项目实现了用于最优无功功率分配（Optimal Reactive Power Dispatch, ORPD）的连续线性逼近算法。该算法能够准确建模离散控制设备（如负荷变压器分接头(LTC)和并联电容器），并通过迭代求解最小化网络损耗。

## 主要特性

- ✅ 支持通用MATPOWER格式电网数据
- ✅ 实现论文中的线性化功率潮流方程
- ✅ 精确建模LTC和并联电容器的离散约束
- ✅ 三层循环迭代求解算法
- ✅ AC可行性验证
- ✅ 高度模块化，易于扩展

## 项目结构

```
SuccessiveLinearOPF/
├── README.md
├── main_SLA_OPF.m              # 主程序入口
├── data/
│   └── ieee14bus_data.m        # IEEE 14节点测试系统
├── core/
│   ├── load_case_data.m        # 电网数据加载
│   ├── initialize_variables.m  # 变量初始化
│   ├── build_admittance_matrix.m # 导纳矩阵构建
│   ├── linearize_pf_equations.m # 功率潮流线性化
│   ├── build_ltc_model.m       # LTC约束建模
│   ├── build_capacitor_model.m # 电容器约束建模
│   └── solve_opf_model.m       # OPF求解器
├── validation/
│   ├── ac_power_flow.m         # AC潮流计算
│   └── verify_solution.m       # 解验证
├── utils/
│   ├── plot_results.m          # 结果可视化
│   └── export_results.m        # 结果导出
└── test/
    └── test_case.m             # 测试脚本
```

## 核心算法

论文中提出的三层循环算法：

- **Loop 1**: 固定离散变量，求解连续ORPD模型（获得高质量初值）
- **Loop 2**: 更新LTC和电容器的离散操作状态
- **Loop 3**: 验证AC可行性并恢复可行解

## 使用方法

```matlab
% 运行主程序
main_SLA_OPF

% 或指定测试系统
[results] = main_SLA_OPF('ieee14bus_data')
```

## 参考文献

Yang, L., et al. "Optimal Reactive Power Dispatch With Accurately Modeled Discrete Control Devices: A Successive Linear Approximation Approach." 
IEEE Transactions on Power Systems, vol. 32, no. 3, May 2017, pp. 2437-2447.

## 开发进度

- [x] 项目结构设计
- [ ] 数据加载模块
- [ ] 线性化模块
- [ ] LTC建模
- [ ] 电容器建模
- [ ] 求解器实现
- [ ] 验证模块
- [ ] 测试用例
