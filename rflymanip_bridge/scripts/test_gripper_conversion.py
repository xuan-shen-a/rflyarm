#!/usr/bin/env python3
"""
Gripper接口转换验证脚本
验证原始编码值与角度(度)之间的转换是否正确
"""

# 转换常数
SERVO_ZERO_POINT = 2048
DEG_TO_RAW_RATIO = 1.0 / 0.0879  # ≈ 11.376
RAW_TO_DEG_RATIO = 0.0879

def raw_to_deg(raw_value):
    """原始编码值 → 角度(度)"""
    return (raw_value - SERVO_ZERO_POINT) * RAW_TO_DEG_RATIO

def deg_to_raw(angle_deg):
    """角度(度) → 原始编码值"""
    return angle_deg * DEG_TO_RAW_RATIO + SERVO_ZERO_POINT

def test_conversion():
    """测试转换函数"""
    print("="*70)
    print("Gripper舵机接口转换验证")
    print("="*70)

    # 测试用例: (原始值, 期望角度)
    test_cases = [
        (2048, 0.0),      # 零点
        (2894, 74.4),     # 你的实际数据
        (2059, 1.0),      # 你的实际数据
        (1672, -33.1),    # 你的实际数据
        (3072, 90.0),     # 90度
        (1024, -90.0),    # -90度
        (4096, 180.0),    # 180度
        (0, -180.0),      # -180度
    ]

    print("\n1. 原始值 → 角度(度) 转换测试:")
    print("-"*70)
    print(f"{'原始编码值':<12} | {'计算角度(度)':>12} | {'期望角度(度)':>12} | {'误差':>8}")
    print("-"*70)

    for raw_value, expected_deg in test_cases:
        calculated_deg = raw_to_deg(raw_value)
        error = abs(calculated_deg - expected_deg)
        status = "✓" if error < 0.2 else "✗"
        print(f"{raw_value:<12} | {calculated_deg:12.2f} | {expected_deg:12.2f} | {error:7.2f} {status}")

    print("\n2. 角度(度) → 原始值 转换测试:")
    print("-"*70)
    print(f"{'角度(度)':<12} | {'计算原始值':>12} | {'期望原始值':>12} | {'误差':>8}")
    print("-"*70)

    for expected_raw, angle_deg in test_cases:
        calculated_raw = deg_to_raw(angle_deg)
        error = abs(calculated_raw - expected_raw)
        status = "✓" if error < 1.0 else "✗"
        print(f"{angle_deg:<12.2f} | {calculated_raw:12.1f} | {expected_raw:12} | {error:7.1f} {status}")

    print("\n3. 双向转换测试 (往返精度):")
    print("-"*70)
    print(f"{'原始输入':<12} | {'→角度→原始':>15} | {'误差':>8}")
    print("-"*70)

    test_raw_values = [0, 1024, 2048, 3072, 4096]
    for raw in test_raw_values:
        deg = raw_to_deg(raw)
        raw_back = deg_to_raw(deg)
        error = abs(raw_back - raw)
        status = "✓" if error < 1.0 else "✗"
        print(f"{raw:<12} | {raw_back:15.1f} | {error:7.1f} {status}")

    print("\n4. 常用角度对照表:")
    print("-"*70)
    print(f"{'角度(度)':<12} | {'原始编码值':>15}")
    print("-"*70)

    common_angles = [-180, -90, -45, 0, 45, 90, 135, 180]
    for angle in common_angles:
        raw = deg_to_raw(angle)
        print(f"{angle:<12} | {raw:15.1f}")

    print("\n5. 转换公式验证:")
    print("-"*70)
    print(f"零点编码值 (SERVO_ZERO_POINT): {SERVO_ZERO_POINT}")
    print(f"角度→原始 系数 (DEG_TO_RAW_RATIO): {DEG_TO_RAW_RATIO:.6f}")
    print(f"原始→角度 系数 (RAW_TO_DEG_RATIO): {RAW_TO_DEG_RATIO:.6f}")
    print(f"验证: DEG_TO_RAW * RAW_TO_DEG = {DEG_TO_RAW_RATIO * RAW_TO_DEG_RATIO:.6f} (应为1.0)")
    print("="*70)

if __name__ == '__main__':
    test_conversion()
