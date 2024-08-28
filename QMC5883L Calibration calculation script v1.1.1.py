def calculate_offsets_and_scales(xmax, xmin, ymax, ymin):
    # 计算偏移量
    offset_x = (xmax + xmin) / 2
    offset_y = (ymax + ymin) / 2

    range_x = xmax - xmin
    range_y = ymax - ymin
    
    # 检查范围是否为零
    if range_y == 0:
        return None  # 返回 None 以表示出错

    # 计算缩放因子
    scale_x = 1
    scale_y = range_x / range_y

    return offset_x, offset_y, scale_x, scale_y

def main():
    # 语言选择
    language = input("请选择语言 / Please select language (en/ch): ").strip().lower()
    
    if language == 'en':
        messages = {
            "author": "Author: Vegetable-SYC\n",
            "version": "Version: v1.0.1\n",
            "date": "Date: 2024/8/28\n",
            "description": "This script calculates the offsets and scales for HMC5883L. Please calculate the max and min values for x and y axes in advance.\n",
            "scale_range": "The scale factor ranges from 0 to 1. Please input the corresponding values below.\n",
            "calibration": '''Calibration method for QMC5883L: Before calibration, place the HMC5883L on a horizontal plane. Read the magnetic field data for x and y axes in the program. After successful upload, wait for a while to stabilize the sensor data, then continuously rotate the HMC5883L in the horizontal direction. After a while, record the max and min values for x and y axes during this period, and input them into the script!''',
            "input_warning": '''Before inputting, note that the max and min values for y axis cannot be equal, otherwise please re-enter.\n''',
            "prompt_xmax": "Please enter xmax: ",
            "prompt_xmin": "Please enter xmin: ",
            "prompt_ymax": "Please enter ymax: ",
            "prompt_ymin": "Please enter ymin: ",
            "error_input": "Invalid input, please ensure ymax is not equal to ymin.\n",
            "results": "************************************************************\n\nOffsets: Offset_x = %.2f, Offset_y = %.2f\nScale factors: Scale_x = %.2f, Scale_y = %.2f\n\n************************************************************",
            "continue_prompt": "Press Enter to continue calculating, or input 'q' to quit: "
        }
    else:
        messages = {
            "author": "作者 : Vegetable-SYC\n",
            "version": "版本 : v1.0.1\n",
            "date": "日期 : 2024/8/28\n",
            "description": "本脚本用于计算HMC5883L的偏移量和缩放因子，需要提前计算出x，y轴的最大最小值\n",
            "scale_range": "得到的缩放因子范围为0~1,请在下方输入对应的值\n",
            "calibration": '''以下提供QMC5883L的校准方法，校准前需要先将HMC5883L置于水平面上，在程序中读取磁场x,y轴的数据。程序上传成功后，先等待一段时间，让传感器的数据先稳定下来，紧接着在水平方向持续转动HMC5883L，转动一段时间后，记录在这段时间里，x，y轴的最大值和最小值，并输入脚本中！''',
            "input_warning": '''在输入前请注意，输入的数据中，且y轴的最大值和最小值不能相等，否则需要重新输入。\n''',
            "prompt_xmax": "请输入 xmax: ",
            "prompt_xmin": "请输入 xmin: ",
            "prompt_ymax": "请输入 ymax: ",
            "prompt_ymin": "请输入 ymin: ",
            "error_input": "输入不规范，请确保 ymax 不等于 ymin。\n",
            "results": "************************************************************\n\n偏移量: Offset_x = %.2f, Offset_y = %.2f\n缩放因子: Scale_x = %.2f, Scale_y = %.2f\n\n************************************************************",
            "continue_prompt": "按下回车键继续计算，或输入 'q' 退出："
        }

    print(messages["author"])
    print(messages["version"])
    print(messages["date"])
    print(messages["description"])
    print(messages["scale_range"])
    print(messages["calibration"])
    print(messages["input_warning"])
    
    while True:
        while True:
            try:
                xmax = float(input(messages["prompt_xmax"]))
                xmin = float(input(messages["prompt_xmin"]))
                ymax = float(input(messages["prompt_ymax"]))
                ymin = float(input(messages["prompt_ymin"]))

                # 计算偏移量和缩放因子
                results = calculate_offsets_and_scales(xmax, xmin, ymax, ymin)

                # 检查是否有错误
                if results is None:
                    print(messages["error_input"])
                    continue  # 重新输入

                offset_x, offset_y, scale_x, scale_y = results

                # 输出结果
                print(messages["results"] % (offset_x, offset_y, scale_x, scale_y))
                break  # 输入正确，退出循环

            except ValueError:
                if language == 'en':
                    print("Invalid input, please enter numbers.\n")  # 捕获输入错误
                else:
                    print("无效输入，请输入数字。\n")

        # 提示用户按下回车继续
        user_input = input(messages["continue_prompt"])
        if user_input.strip().lower() == 'q':
            break  # 用户选择退出

if __name__ == "__main__":
    main()
