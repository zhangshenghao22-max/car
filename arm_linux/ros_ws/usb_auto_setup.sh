#!/bin/bash

# USB设备自动设置脚本
# 第一次运行：记录设备描述符并保存到 usb.desc
# 后续运行：读取 usb.desc 文件自动识别和设置设备

DESC_FILE="usb.desc"
declare -A DESC_MAP    # 从文件读取的映射：设备名->描述符
declare -A REV_MAP     # 反向映射：描述符->设备名（用于快速查找）
declare -A CUR_MAP     # 当前运行的映射：实际设备->设备名

# 默认设备名（用于首次运行时的顺序映射）
declare -a ACM_DEVICE_NAMES=("imu" "laser")
declare -a USB_DEVICE_NAMES=("rt_shell")

# 函数：获取USB设备描述符（厂商ID:产品ID:序列号）
get_usb_descriptor() {
    local device=$1
    local device_name=$(basename "$device")
    
    # 根据设备类型确定sysfs路径
    if [[ "$device_name" == ttyACM* ]]; then
        local sysfs_path="/sys/class/tty/$device_name"
    elif [[ "$device_name" == ttyUSB* ]]; then
        local sysfs_path="/sys/class/tty/$device_name"
    else
        local sysfs_path="/sys/class/tty/$device_name"
    fi
    
    # 方法1：通过sysfs获取（最可靠）
    if [[ -d "$sysfs_path" ]]; then
        # 查找USB设备信息
        local usb_path=$(find "$sysfs_path" -name "idVendor" -o -name "serial" | head -1 | xargs dirname 2>/dev/null)
        if [[ -d "$usb_path" ]]; then
            local vendor_id=$(cat "$usb_path/idVendor" 2>/dev/null 2>/dev/null | tr -d '\n\r ' | tr '[:lower:]' '[:upper:]')
            local product_id=$(cat "$usb_path/idProduct" 2>/dev/null 2>/dev/null | tr -d '\n\r ' | tr '[:lower:]' '[:upper:]')
            local serial=$(cat "$usb_path/serial" 2>/dev/null 2>/dev/null | tr -d '\n\r ')
            
            # 清理十六进制前缀
            vendor_id=${vendor_id#0x}
            product_id=${product_id#0x}
            
            if [[ -n "$vendor_id" && -n "$product_id" ]]; then
                echo "${vendor_id}:${product_id}:${serial:-unknown}"
                return 0
            fi
        fi
    fi
    
    # 方法2：使用udevadm
    local udev_info=$(udevadm info -q property -n "$device" 2>/dev/null)
    if [[ -n "$udev_info" ]]; then
        local vendor_id=$(echo "$udev_info" | grep -i "ID_VENDOR_ID" | head -1 | cut -d= -f2 | tr -d '\n\r ')
        local product_id=$(echo "$udev_info" | grep -i "ID_MODEL_ID" | head -1 | cut -d= -f2 | tr -d '\n\r ')
        local serial=$(echo "$udev_info" | grep -i "ID_SERIAL_SHORT" | head -1 | cut -d= -f2 | tr -d '\n\r ')
        
        if [[ -n "$vendor_id" && -n "$product_id" ]]; then
            vendor_id=${vendor_id#0x}
            product_id=${product_id#0x}
            echo "${vendor_id^^}:${product_id^^}:${serial:-unknown}"
            return 0
        fi
    fi
    
    echo ""
    return 1
}

# 函数：加载描述符文件
load_desc_file() {
    if [[ -f "$DESC_FILE" ]]; then
        echo "读取描述符文件: $DESC_FILE"
        local line_num=0
        
        while IFS='=' read -r device_name descriptor; do
            line_num=$((line_num + 1))
            
            # 跳过空行和注释
            [[ -z "$device_name" ]] && continue
            [[ "$device_name" =~ ^[[:space:]]*# ]] && continue
            
            # 清理空白字符
            device_name=$(echo "$device_name" | tr -d '[:space:]')
            descriptor=$(echo "$descriptor" | tr -d '\n\r')
            
            if [[ -z "$descriptor" ]]; then
                echo "  警告: 第${line_num}行描述符为空，跳过"
                continue
            fi
            
            # 存储映射
            DESC_MAP["$device_name"]="$descriptor"
            REV_MAP["$descriptor"]="$device_name"
            
            echo "  找到映射: $device_name -> $descriptor"
            
        done < "$DESC_FILE"
        
        if [[ ${#DESC_MAP[@]} -gt 0 ]]; then
            return 0
        else
            echo "  错误: 描述符文件为空或格式错误"
            return 1
        fi
    fi
    
    return 1
}

# 函数：创建设备符号链接
create_device_link() {
    local device=$1
    local target_name=$2
    
    echo "  创建链接: /dev/$target_name -> $device"
    
    # 移除已存在的链接
    sudo rm -f "/dev/$target_name" 2>/dev/null
    
    # 创建新链接
    if sudo ln -sf "$device" "/dev/$target_name"; then
        # 设置链接权限
        sudo chmod 666 "/dev/$target_name" 2>/dev/null || sudo chmod 777 "/dev/$target_name"
        return 0
    else
        echo "  错误: 创建符号链接失败"
        return 1
    fi
}

# 函数：保存描述符到文件
save_desc_file() {
    echo "保存设备描述符到文件: $DESC_FILE"
    
    # 创建文件头
    cat > "$DESC_FILE" << EOF
# USB设备描述符映射文件
# 生成时间: $(date)
# 格式: 设备名=厂商ID:产品ID:序列号
# 注意: 请勿手动修改此文件

EOF
    
    # 写入设备映射（按设备类型分组排序）
    # 先写ACM设备（imu, laser）
    for device_name in imu laser; do
        if [[ -n "${DESC_MAP[$device_name]}" ]]; then
            echo "$device_name=${DESC_MAP[$device_name]}" >> "$DESC_FILE"
        fi
    done
    
    # 再写USB设备（rt_shell）
    for device_name in rt_shell; do
        if [[ -n "${DESC_MAP[$device_name]}" ]]; then
            echo "$device_name=${DESC_MAP[$device_name]}" >> "$DESC_FILE"
        fi
    done
    
    echo "文件已保存，包含 ${#DESC_MAP[@]} 个设备映射"
}

# 函数：检测所有USB设备
detect_devices() {
    local -n dev_list=$1
    dev_list=()
    
    # 检测ACM设备
    for dev in /dev/ttyACM*; do
        [[ -e "$dev" ]] && dev_list+=("$dev")
    done
    
    # 检测USB设备
    for dev in /dev/ttyUSB*; do
        [[ -e "$dev" ]] && dev_list+=("$dev")
    done
}

# 主程序开始
echo "=========================================="
echo "      USB设备自动配置脚本"
echo "=========================================="

# 检测所有USB设备
ALL_DEVICES=()
detect_devices ALL_DEVICES

if [[ ${#ALL_DEVICES[@]} -eq 0 ]]; then
    echo "错误: 未检测到任何USB设备 (ttyACM* 或 ttyUSB*)"
    echo "请插入设备后重试"
    exit 1
fi

# 分类统计设备
ACM_COUNT=0
USB_COUNT=0
for dev in "${ALL_DEVICES[@]}"; do
    if [[ "$(basename "$dev")" == ttyACM* ]]; then
        ACM_COUNT=$((ACM_COUNT + 1))
    elif [[ "$(basename "$dev")" == ttyUSB* ]]; then
        USB_COUNT=$((USB_COUNT + 1))
    fi
done

echo "检测到设备:"
echo "  ACM设备 (ttyACM*): $ACM_COUNT 个"
echo "  USB设备 (ttyUSB*): $USB_COUNT 个"
echo ""

# 检查是否为第一次运行
FIRST_RUN=false
if [[ -f "$DESC_FILE" ]]; then
    echo "发现描述符文件，进入自动识别模式"
    echo "------------------------------------------"
    
    # 加载描述符文件
    if ! load_desc_file; then
        echo "描述符文件加载失败，将重新创建设备映射"
        FIRST_RUN=true
    else
        echo "成功加载 ${#DESC_MAP[@]} 个设备映射"
    fi
else
    echo "未找到描述符文件，进入首次运行模式"
    echo "------------------------------------------"
    echo "重要提示: 首次运行需要按顺序插入设备"
    echo "插入顺序:"
    echo "  1. IMU设备 (将映射为 /dev/imu)"
    echo "  2. 激光雷达设备 (将映射为 /dev/laser)"
    echo "  3. RT Shell设备 (将映射为 /dev/rt_shell)"
    echo ""
    echo "请确保设备已按正确顺序插入，然后按回车键继续..."
    read -r
    FIRST_RUN=true
fi

echo ""

# 为不同类型设备分别计数
ACM_INDEX=0
USB_INDEX=0
SUCCESS_COUNT=0

# 处理每个设备
for device in "${ALL_DEVICES[@]}"; do
    device_basename=$(basename "$device")
    device_type=""
    
    # 确定设备类型
    if [[ "$device_basename" == ttyACM* ]]; then
        device_type="ACM"
    elif [[ "$device_basename" == ttyUSB* ]]; then
        device_type="USB"
    else
        echo "跳过未知设备类型: $device_basename"
        continue
    fi
    
    echo "处理${device_type}设备: $device_basename"
    
    # 获取设备描述符
    descriptor=$(get_usb_descriptor "$device")
    if [[ -z "$descriptor" ]]; then
        echo "  错误: 无法获取设备描述符"
        continue
    fi
    
    echo "  描述符: $descriptor"
    
    # 根据运行模式确定设备名
    if [[ "$FIRST_RUN" == "true" ]]; then
        # 首次运行：按顺序分配设备名
        if [[ "$device_type" == "ACM" && $ACM_INDEX -lt ${#ACM_DEVICE_NAMES[@]} ]]; then
            target_name="${ACM_DEVICE_NAMES[$ACM_INDEX]}"
            DESC_MAP["$target_name"]="$descriptor"
            REV_MAP["$descriptor"]="$target_name"
            echo "  首次映射: $device_basename -> $target_name"
            ACM_INDEX=$((ACM_INDEX + 1))
        elif [[ "$device_type" == "USB" && $USB_INDEX -lt ${#USB_DEVICE_NAMES[@]} ]]; then
            target_name="${USB_DEVICE_NAMES[$USB_INDEX]}"
            DESC_MAP["$target_name"]="$descriptor"
            REV_MAP["$descriptor"]="$target_name"
            echo "  首次映射: $device_basename -> $target_name"
            USB_INDEX=$((USB_INDEX + 1))
        else
            echo "  警告: ${device_type}设备超出预期数量，跳过"
            continue
        fi
    else
        # 后续运行：根据描述符查找设备名
        target_name="${REV_MAP[$descriptor]}"
        if [[ -z "$target_name" ]]; then
            echo "  错误: 未知设备，未在描述符文件中找到匹配"
            continue
        fi
        echo "  自动识别: $device_basename -> $target_name"
    fi
    
    # 设置设备权限
    echo "  设置权限..."
    if sudo chmod 666 "$device" 2>/dev/null; then
        echo "    ✓ 权限: 666"
    elif sudo chmod 777 "$device" 2>/dev/null; then
        echo "    ✓ 权限: 777"
    else
        echo "    ✗ 权限设置失败"
        continue
    fi
    
    # 创建设备链接
    if create_device_link "$device" "$target_name"; then
        CUR_MAP["$device_basename"]="$target_name"
        SUCCESS_COUNT=$((SUCCESS_COUNT + 1))
    fi
    
    echo ""
done

# 如果是首次运行且成功处理了设备，保存描述符文件
if [[ "$FIRST_RUN" == "true" && ${#DESC_MAP[@]} -gt 0 ]]; then
    echo "------------------------------------------"
    save_desc_file
    echo ""
    echo "首次运行完成！设备信息已保存。"
    echo "下次运行时，设备插入顺序不再重要。"
fi

echo "=========================================="
echo "              配置结果"
echo "=========================================="

# 显示配置结果
if [[ $SUCCESS_COUNT -gt 0 ]]; then
    echo "成功配置 $SUCCESS_COUNT 个设备:"
    for dev in "${!CUR_MAP[@]}"; do
        echo "  /dev/$dev -> /dev/${CUR_MAP[$dev]}"
    done
else
    echo "未成功配置任何设备"
fi

echo ""
echo "设备链接状态:"
# 检查所有目标设备
TARGET_DEVICES=("imu" "laser" "rt_shell")
for dev_name in "${TARGET_DEVICES[@]}"; do
    if [[ -L "/dev/$dev_name" ]]; then
        link_target=$(readlink -f "/dev/$dev_name" 2>/dev/null || echo "未知")
        echo "  /dev/$dev_name -> $link_target"
    else
        echo "  /dev/$dev_name: 未创建"
    fi
done

echo ""
echo "原始设备状态:"
# 显示所有检测到的原始设备
for dev_type in "ttyACM" "ttyUSB"; do
    for dev in /dev/${dev_type}*; do
        if [[ -e "$dev" ]]; then
            perms=$(stat -c "%A" "$dev" 2>/dev/null || echo "未知")
            echo "  $dev ($perms)"
        fi
    done
done

# 显示描述符文件信息
if [[ -f "$DESC_FILE" ]]; then
    echo ""
    echo "描述符文件 ($DESC_FILE):"
    echo "------------------------------------------"
    cat "$DESC_FILE" | grep -v "^#" | while read line; do
        [[ -n "$line" ]] && echo "  $line"
    done
fi

echo "=========================================="
echo "脚本执行完成"
echo "=========================================="
