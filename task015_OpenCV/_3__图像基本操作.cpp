/********************************************************************
 * @author @b    YinKang'an
 * @date         2026.01.29
 * @brief        核心数据结构
 * ------------------------------------------------------------------
 * @attention @b [声明说明]
 * \b            本代码中重写的中文注释+函数声明，仅用于鼠标悬停查看中文说明，直接编译`可能`会因类型别名导致冲突报错；
 * \b            若需实际编译运行，可参考下面方法：
 * 
 * \b            -删掉在本文件下方声明，仅保留中文注释
 * \b            -调用时用cv::函数名(会显示官方英文注释)
 * 
 * \b            某些OpenCV函数可直接仿照本注释格式添加中文说明，因参数类型无歧义，不会触发编译冲突
 * ------------------------------------------------------------------
 * @note         所有函数都重写了中文注释,鼠标光标移动到参数名上即可查看中文注释
 *               移动到函数名上可查看原始官方英文注释
 *               移动到参数类型上可查看原始官方英文参数类型
 ********************************************************************/

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

// 1.结构体封装数据
struct TrackbarParams {
  Mat src_img;          /**< 源图像 Source image */
  Mat dst_img;          /**< 结果图像 Result image */
  string win_name;      /**< 窗口名 Window name */
};






// 2.自己写的回调函数,userdata用来传参
void onAdjustBrightContrast(int pos, void* userdata) {
  TrackbarParams* p_params = (TrackbarParams*)userdata;
 

//--------------------------------------------------------------------------
/**
 * @brief 获取轨迹条当前位置
 * 
 * 该函数返回指定轨迹条的当前位置值，是读取轨迹条实时状态的核心函数。
 * 
 * @param trackbarname [in] 轨迹条的名称（创建时指定的唯一标识）
 * @param winname [in] 作为轨迹条父窗口的窗口名称
 * @return int 轨迹条的当前位置值；若轨迹条/窗口不存在，返回-1
 * 
 * @note 1. [仅Qt后端生效] 若轨迹条附加到控制面板，winname 参数可设为空字符串；
 *       2. 返回值范围与创建轨迹条时指定的 count 参数一致（0 ~ count）；
 *       3. 调用前需确保轨迹条已通过 createTrackbar 创建，否则返回无效值；
 *       4. 该函数为实时读取，可在回调函数外获取轨迹条最新位置，无需依赖回调传值。
 * @sa cv::createTrackbar, cv::setTrackbarPos, cv::namedWindow
 */
CV_EXPORTS_W int getTrackbarPos(const String& trackbarname, const String& winname);
//--------------------------------------------------------------------------
  // 获取滑动条当前值
  int brightness = cv::getTrackbarPos("Brightness", p_params->win_name);
  int contrast = cv::getTrackbarPos("Contrast", p_params->win_name);
//--------------------------------------------------------------------------


  // 亮度对比度计算
  float alpha = contrast / 50.0;  // 对比度系数 Contrast coefficient
  int beta = brightness - 50;     // 亮度增量 Brightness increment


//--------------------------------------------------------------------------
/**
 * @brief 将数组转换为另一种数据类型（可选缩放）
 * 
 * 该方法将源矩阵的像素值转换为目标数据类型，最终会应用 saturate_cast<> 函数避免可能的数值溢出，
 * 转换公式如下：
 * \f[m(x,y) = saturate \_ cast<rType>( \alpha (*this)(x,y) + \beta )\f]
 * 
 * @param m [out] 输出矩阵；若操作前矩阵尺寸或类型不符合要求，会自动重新分配内存
 * @param rtype [in] 期望的输出矩阵类型（实际为深度值，通道数与输入矩阵保持一致）；
 *        若 rtype 为负数，输出矩阵类型与输入矩阵完全相同
 * @param alpha [in] 可选缩放因子，默认值为1.0 (把原图像的每个像素值都乘以这个数)
 * @param beta [in] 可选增量值，会添加到缩放后的像素值中，默认值为0.0 (把原图像的每个像素值都加上这个数)
 * 
 * @note 1. saturate_cast<> 函数会将超出目标数据类型范围的值截断到合法区间（如8位无符号类型会截断到0~255）；
 *       2. 通道数始终保持不变，仅改变每个通道的数据深度（如CV_8UC3转CV_32FC3，通道数仍为3）；
 *       3. 常用场景：灰度图像归一化（alpha=1/255.0, beta=0 转CV_8UC1为CV_32FC1）、数值缩放增强对比度；
 *       4. 若 rtype 设为-1，等价于直接复制矩阵（无类型转换），仅 alpha 和 beta 会作用于像素值。
 * @sa cv::saturate_cast, cv::Mat::type, cv::Mat::depth, cv::normalize
 */
void convertTo(cv::OutputArray m, int rtype, double alpha = (1.0), double beta = (0.0));
//--------------------------------------------------------------------------
  // 图像处理
  p_params->src_img.convertTo(p_params->dst_img, -1, alpha, beta);
//--------------------------------------------------------------------------


  Mat watermark;


//--------------------------------------------------------------------------
/**
 * @brief 将图像缩放到指定尺寸
 * 
 * 该函数将源图像 src 缩小或放大到指定尺寸，注意初始的 dst 类型和尺寸不会被考虑；
 * 输出图像的尺寸和类型由 src、dsize、fx、fy 共同决定。
 * 若希望将 src 缩放到预创建的 dst 尺寸，可按如下方式调用：
 * @code
 *     // 显式指定 dsize=dst.size()，fx 和 fy 会据此自动计算
 *     resize(src, dst, dst.size(), 0, 0, interpolation);
 * @endcode
 * 若希望将图像在每个方向上缩小2倍，可按如下方式调用：
 * @code
 *     // 指定 fx 和 fy，函数会自动计算目标图像尺寸
 *     resize(src, dst, Size(), 0.5, 0.5, interpolation);
 * @endcode
 * 
 * 缩小图像时，使用 INTER_AREA 插值方式效果最佳；
 * 放大图像时，使用 INTER_CUBIC（速度慢）或 INTER_LINEAR（速度快且效果尚可）插值方式效果最佳。
 * 
 * @param src [in] 输入图像
 * @param dst [out] 输出图像；尺寸为 dsize（非0时）或由 src.size()、fx、fy 计算得出；类型与 src 一致
 * @param dsize [in] 输出图像尺寸；若等于0（Python中为None），按如下公式计算：
 *        \f[\texttt{dsize = Size(round(fx*src.cols), round(fy*src.rows))}\f]
 *        dsize 和（fx、fy）两者中必须至少有一个非0
 * @param fx [in] 水平轴缩放因子；若等于0，按如下公式计算：
 *        \f[\texttt{(double)dsize.width/src.cols}\f]
 * @param fy [in] 垂直轴缩放因子；若等于0，按如下公式计算：
 *        \f[\texttt{(double)dsize.height/src.rows}\f]
 * @param interpolation [in] 插值方法，取值为 InterpolationFlags 枚举类型，默认值为 INTER_LINEAR
 * 
 * @note 1. 插值方法选择原则：
 *          - 缩小图像：优先使用 INTER_AREA（区域插值），可避免锯齿；
 *          - 放大图像：优先使用 INTER_CUBIC（双三次插值，效果好但慢）或 INTER_LINEAR（双线性插值，速度快）；
 *          - 快速预览：可用 INTER_NEAREST（最近邻插值，最快但效果最差）；
 *       2. dsize 和 fx/fy 二选一即可，若同时指定，dsize 优先级更高（fx/fy 会被覆盖）；
 *       3. 缩放因子 fx/fy 支持非等比例缩放（如 fx=0.5, fy=1.0 仅水平方向缩小）；
 *       4. 输出图像 dst 无需提前分配内存，函数会自动创建匹配的尺寸和类型。
 * @sa cv::warpAffine, cv::warpPerspective, cv::remap, cv::InterpolationFlags
 */
CV_EXPORTS_W void resize( InputArray src, OutputArray dst, Size dsize, double fx = 0, double fy = 0, int interpolation = INTER_LINEAR );
//--------------------------------------------------------------------------
  cv::resize(p_params->src_img, watermark, Size(p_params->src_img.cols/4, p_params->src_img.rows/4));
  Rect roi(10, 10, watermark.cols, watermark.rows);
//--------------------------------------------------------------------------


//--------------------------------------------------------------------------
/**
 * @brief 计算两个数组的加权和
 * 
 * 该函数按如下公式计算两个数组的加权和：
 * \f[\texttt{dst} (I)= \texttt{saturate} ( \texttt{src1} (I)* \texttt{alpha} +  \texttt{src2} (I)* \texttt{beta} +  \texttt{gamma} )\f]
 * 其中 I 是数组元素的多维索引。对于多通道数组，每个通道会独立处理。
 * 该函数可等价替换为矩阵表达式：
 * @code{.cpp}
 *     dst = src1*alpha + src2*beta + gamma;
 * @endcode
 * 
 * @param src1 [in] 第一个输入数组
 * @param alpha [in] 第一个数组元素的权重
 * @param src2 [in] 第二个输入数组，需与 src1 尺寸和通道数完全相同
 * @param beta [in] 第二个数组元素的权重
 * @param gamma [in] 加到每个加权和上的标量值
 * @param dst [out] 输出数组，尺寸和通道数与输入数组一致
 * @param dtype [in] 输出数组的可选深度；当两个输入数组深度相同时，可设为-1（等价于 src1.depth()）
 * 
 * @note 1. 当输出数组深度为 CV_32S（32位整型）时，不会应用 saturate 饱和截断，
 *          溢出时甚至会得到符号错误的结果；
 *       2. 核心应用场景：图像融合（如两张图片渐变叠加），alpha + beta 通常设为1.0以保证亮度不变；
 *       3. 输入数组必须尺寸、通道数完全一致，否则会报维度不匹配错误；
 *       4. saturate 函数会将结果截断到目标数据类型的合法范围（如8位无符号类型截断到0~255）。
 * @sa cv::add, cv::subtract, cv::scaleAdd, cv::Mat::convertTo, cv::saturate_cast
 */
//CV_EXPORTS_W void addWeighted(InputArray src1, double alpha, InputArray src2, double beta, double gamma, OutputArray dst, int dtype = -1);
//--------------------------------------------------------------------------
  addWeighted(p_params->dst_img(roi), 0.8, watermark, 0.2, 0, p_params->dst_img(roi));      /**< 目标ROI = 原ROI*0.8 + 水印*0.2 + 0 */
//--------------------------------------------------------------------------


//--------------------------------------------------------------------------
/**
 * @brief 在指定窗口显示图像
 * 
 * 该函数在指定窗口中显示图像。若窗口以 cv::WINDOW_AUTOSIZE 标志创建，图像会以原始尺寸显示（仍受屏幕分辨率限制）；
 * 否则图像会缩放至适配窗口大小。函数会根据图像深度对像素值进行缩放处理：
 * 
 * - 若图像为8位无符号类型，直接按原始像素值显示；
 * - 若图像为16位无符号类型，像素值会除以256（即 [0,255×256] 映射到 [0,255]）；
 * - 若图像为32位/64位浮点型，像素值会乘以256（即 [0,1] 映射到 [0,255]）；
 * - 32位整型图像不再被处理（因转换规则不明确），需根据图像场景通过自定义预处理转换为8位无符号矩阵。
 * 
 * 若窗口以OpenGL支持创建，cv::imshow 还支持 ogl::Buffer、ogl::Texture2D 和 cuda::GpuMat 作为输入。
 * 若调用此函数前未创建窗口，会默认以 cv::WINDOW_AUTOSIZE 标志创建窗口。
 * 若需显示超过屏幕分辨率的图像，需在 imshow 前调用 namedWindow("", WINDOW_NORMAL)。
 * 
 * @param winname [in] 窗口名称
 * @param mat [in] 待显示的图像矩阵
 * 
 * @note 1. 该函数调用后必须配合 cv::waitKey 或 cv::pollKey 使用，以完成GUI后台任务，
 *          才能真正显示图像并让窗口响应鼠标/键盘事件；否则图像无法显示，窗口可能卡死：
 *          - waitKey(0)：无限显示窗口，直至按下任意键（适合静态图像显示）；
 *          - waitKey(25)：显示帧并等待约25毫秒（适合逐帧显示视频）；
 *          如需关闭窗口，调用 cv::destroyWindow；
 *       2. [仅Windows后端生效] 按下 Ctrl+C 可将图像复制到剪贴板；
 *       3. [仅Windows后端生效] 按下 Ctrl+S 会弹出保存图像的对话框；
 *       4. 显示前需确保 mat 非空矩阵，否则窗口会显示空白。
 * @sa cv::namedWindow, cv::waitKey, cv::pollKey, cv::destroyWindow, cv::imread
 */
CV_EXPORTS_W void imshow(const String& winname, InputArray mat);
//--------------------------------------------------------------------------
  cv::imshow(p_params->win_name, p_params->dst_img);
//--------------------------------------------------------------------------
}











//__________________________________________________________________________
//__________________________________________________________________________
int main() {
//1.图像读取

//--------------------------------------------------------------------------
/**
 * @brief 从文件加载图像
 * 
 * 该函数从指定文件中加载图像并返回图像矩阵；若图像无法读取（文件缺失、权限不足、格式不支持/无效），
 * 函数会返回空矩阵（Mat::data==NULL）。
 * 
 * 当前支持的文件格式如下：
 * - Windows位图 - *.bmp、*.dib（始终支持）
 * - JPEG文件 - *.jpeg、*.jpg、*.jpe（见Note部分）
 * - JPEG 2000文件 - *.jp2（见Note部分）
 * - 便携式网络图形 - *.png（见Note部分）
 * - WebP - *.webp（见Note部分）
 * - 便携式图像格式 - *.pbm、*.pgm、*.ppm、*.pxm、*.pnm（始终支持）
 * - PFM文件 - *.pfm（见Note部分）
 * - Sun光栅 - *.sr、*.ras（始终支持）
 * - TIFF文件 - *.tiff、*.tif（见Note部分）
 * - OpenEXR图像文件 - *.exr（见Note部分）
 * - Radiance HDR - *.hdr、*.pic（始终支持）
 * - GDAL支持的栅格和矢量地理空间数据（见Note部分）
 * 
 * @param filename [in] 待加载文件的路径和名称
 * @param flags [in] 图像读取模式标志，取值为cv::ImreadModes枚举类型，默认值为IMREAD_COLOR
 * @return Mat 加载成功返回存储图像数据的矩阵；加载失败返回空矩阵（Mat::data==NULL）
 * 
 * @note 1. 函数通过文件内容判断图像类型，而非文件扩展名（如改后缀为.jpg的txt文件仍无法读取）；
 *       2. 彩色图像解码后通道按 **B G R** 顺序存储（非常见的RGB）；
 *       3. 使用IMREAD_GRAYSCALE时，优先使用编解码器内置灰度转换（若支持），结果可能与cvtColor()不同；
 *       4. Windows/MacOSX系统默认使用OpenCV自带编解码器（libjpeg/libpng等），可稳定读取JPEG/PNG/TIFF；
 *          MacOSX可选原生读取器，但像素值可能因系统色彩管理与自带编解码器不同；
 *       5. Linux/BSD等类Unix系统需安装编解码器开发包（如Debian/Ubuntu的libjpeg-dev），或在CMake中开启
 *          OPENCV_BUILD_3RDPARTY_LIBS以启用OpenCV自带编解码器；
 *       6. CMake开启WITH_GDAL且使用IMREAD_LOAD_GDAL标志时，会调用GDAL驱动解码地理空间数据；
 *       7. 图像含EXIF方向信息时会自动旋转（传入IMREAD_IGNORE_ORIENTATION/IMREAD_UNCHANGED除外）；
 *       8. 需保留PFM图像浮点值时，需使用IMREAD_UNCHANGED标志；
 *       9. 默认图像像素总数不超过2^30，可通过系统变量OPENCV_IO_MAX_IMAGE_PIXELS修改限制；
 *       10. 路径含中文时需转UTF-8编码（Windows），否则易加载失败；
 *       11. 常用flags取值：
 *           - IMREAD_COLOR（默认）：加载彩色图，忽略透明通道（3通道BGR）；
 *           - IMREAD_GRAYSCALE：加载灰度图（单通道）；
 *           - IMREAD_UNCHANGED：加载原图所有通道（含透明通道）。
 * @sa cv::imwrite, cv::imshow, cv::cvtColor, cv::ImreadModes
 */
CV_EXPORTS_W Mat imread( const String& filename, int flags = IMREAD_COLOR );
//--------------------------------------------------------------------------
  Mat src = cv::imread("./pic.jpg");
//--------------------------------------------------------------------------


//--------------------------------------------------------------------------

  if (src.empty()) {
    cout << "图像读取失败！请检查文件路径是否正确，建议使用英文路径" << endl;
    return -1;
  }

  //结构体赋值
  TrackbarParams params;

  params.src_img = src;
  params.win_name = "Image_Adjust_Tool";

  //创建窗口 + 滑动条

  //--------------------------------------------------------------------------
/**
 * @brief 创建窗口
 * 
 * 该函数创建一个可用于显示图像和放置轨迹条的窗口，创建的窗口通过名称进行标识。
 * 若同名窗口已存在，该函数不执行任何操作。
 * 
 * 你可以调用 cv::destroyWindow 或 cv::destroyAllWindows 关闭窗口并释放相关内存；
 * 对于简单程序，无需显式调用这些函数，因为应用程序退出时操作系统会自动关闭所有资源和窗口。
 * 
 * @param winname [in] 窗口名称（显示在标题栏），可作为窗口的唯一标识符
 * @param flags [in] 窗口属性标志，默认值为 WINDOW_AUTOSIZE，支持的标志（cv::WindowFlags）如下：
 * 
 * \b       WINDOW_NORMAL/WINDOW_AUTOSIZE：
 *          WINDOW_NORMAL 允许手动调整窗口大小；
 *          WINDOW_AUTOSIZE 自动调整窗口尺寸以适配显示的图像（见 imshow），且无法手动修改窗口大小
 * \b       WINDOW_FREERATIO/WINDOW_KEEPRATIO：
 *          WINDOW_FREERATIO 缩放图像时不保持比例；
 *          WINDOW_KEEPRATIO 缩放图像时保持原始比例
 * \b       WINDOW_GUI_NORMAL/WINDOW_GUI_EXPANDED：
 *          WINDOW_GUI_NORMAL 为旧版样式（无状态栏和工具栏）；
 *          WINDOW_GUI_EXPANDED 为新版增强型GUI样式
 * 
 * @note 1. [仅Qt后端生效] 上述扩展标志仅在Qt作为HighGUI后端时支持；
 *       2. 默认标志组合为 WINDOW_AUTOSIZE | WINDOW_KEEPRATIO | WINDOW_GUI_EXPANDED；
 *       3. 窗口名称区分大小写，"Window" 和 "window" 会被识别为两个不同窗口；
 *       4. 创建窗口后需配合 imshow 使用才能显示内容，单独创建窗口不会有可视化效果。
 * @sa cv::destroyWindow, cv::destroyAllWindows, cv::imshow, cv::createTrackbar
 */
CV_EXPORTS_W void namedWindow(const cv::String& winname, int flags = WINDOW_AUTOSIZE);
//--------------------------------------------------------------------------
  cv::namedWindow(params.win_name, WINDOW_NORMAL); // 先创窗口，再创滑动条（关键步骤！）
//--------------------------------------------------------------------------

  // 滑动条参数：滑动条名(英文)、窗口名、初始值地址、最大值、回调函数、userdata传参
  int init_bright = 50;
  int init_contrast = 50;

  
//--------------------------------------------------------------------------
/**
 * @brief 创建轨迹条并附加到指定窗口
 * 
 * 该函数创建一个指定名称和数值范围的轨迹条（滑块/范围控制器），将整型变量 value 与轨迹条位置同步绑定，
 * 并指定轨迹条位置变化时触发的回调函数 onChange。创建后的轨迹条会显示在指定的父窗口 winname 中。
 * 
 * @param trackbarname [in] 创建的轨迹条名称（唯一标识）
 * @param winname [in] 作为轨迹条父窗口的窗口名称
 * @param value [in/out] 可选整型变量指针，其值反映滑块当前位置；轨迹条创建时，滑块初始位置由该变量值决定
 * @param count [in] 滑块的最大位置值，滑块最小位置值固定为 0（取值范围：0 ~ count）
 * @param onChange [in] 滑块位置变化时调用的回调函数指针，默认值为 0（无回调）；
 *        回调函数需符合原型：void Foo(int, void*); 其中第一个参数为轨迹条当前位置，
 *        第二个参数为用户自定义数据（见 userdata 参数）；若该参数为 NULL，仅更新 value 值，不触发回调
 * @param userdata [in] 传递给回调函数的用户自定义数据，默认值为 0；可用于在不使用全局变量的情况下处理轨迹条事件
 * @return int 成功创建返回轨迹条标识符（非负整数）；创建失败返回 -1
 * 
 * @note 1. [仅Qt后端生效] 若需将轨迹条附加到控制面板，winname 参数可设为空字符串；
 *       2. 点击轨迹条的标签可手动输入数值修改轨迹条位置（无需拖动滑块）；
 *       3. value 参数若不为 NULL，需保证其指向的变量生命周期覆盖轨迹条使用周期（否则会导致内存访问错误）；
 *       4. 回调函数 onChange 会在滑块每次拖动/数值修改时触发，第一个参数为实时位置值；
 *       5. userdata 可传递任意类型数据（如结构体、Mat指针等），避免使用全局变量，提升代码封装性；
 *       6. 轨迹条创建前需确保父窗口已通过 namedWindow 创建（除非使用Qt控制面板）。
 * @sa cv::getTrackbarPos, cv::setTrackbarPos, cv::namedWindow, cv::waitKey
 */
CV_EXPORTS int createTrackbar(const String& trackbarname, const String& winname, int* value, int count, TrackbarCallback onChange = 0, void* userdata = 0);
//--------------------------------------------------------------------------
  cv::createTrackbar("Brightness", params.win_name, &init_bright, 100, onAdjustBrightContrast, &params);
  cv::createTrackbar("Contrast",   params.win_name, &init_contrast, 100, onAdjustBrightContrast, &params);
//--------------------------------------------------------------------------


  onAdjustBrightContrast(init_bright, &params);                   /**< 手动触发回调一次,显示初始画面 */
  cout << "滑动条已显示！拖动调节亮度/对比度，按任意键退出并保存图像" << endl;


//--------------------------------------------------------------------------
/**
 * @brief 等待按键按下
 * 
 * 该函数会无限期等待按键事件（当 delay ≤ 0 时），或等待指定的 delay 毫秒（当 delay 为正数时）。
 * 由于操作系统切换线程存在最小时间间隔，函数不会精确等待 delay 毫秒，而是至少等待该时长，
 * 具体等待时间取决于当前计算机运行的其他程序。若指定时间内无按键按下，返回 -1；否则返回按下按键的编码。
 * 若仅需检测按键是否按下但不阻塞等待，可使用 pollKey 函数。
 * 
 * @param delay [in] 等待延迟时间（单位：毫秒），0 是特殊值，表示“无限等待”直至有按键按下
 * @return int 按下按键的ASCII编码（如ESC键返回27、空格键返回32）；指定时间内无按键按下返回 -1
 * 
 * @note 1. waitKey 和 pollKey 是 HighGUI 中唯一能获取并处理 GUI 事件的方法，因此除非 HighGUI
 *          运行在自动处理事件的环境中，否则需定期调用其中一个函数，才能正常显示图像、响应窗口/轨迹条操作；
 *       2. 该函数仅在至少创建一个 HighGUI 窗口且窗口处于激活状态时生效；若存在多个 HighGUI 窗口，
 *          任意一个窗口激活即可（点击窗口使其成为前台窗口）；
 *       3. 线程切换导致等待时间不精确：比如设置 delay=25，实际等待时间可能是25ms~50ms，取决于系统负载；
 *       4. 配合 imshow 使用时，waitKey(0) 可让图像窗口持续显示直至按下任意键（适合静态图像），
 *          waitKey(25) 适合逐帧显示视频（每帧停留约25毫秒）；
 *       5. 该函数无法识别方向键、F1-F12等特殊按键，如需获取这类按键编码，需使用 waitKeyEx 函数。
 * 
 * @sa cv::waitKeyEx, cv::pollKey, cv::imshow, cv::namedWindow
 */
CV_EXPORTS_W int waitKey(int delay = 0);
//--------------------------------------------------------------------------
  cv::waitKey(0); // 阻塞窗口，必须加！否则窗口一闪而过
//--------------------------------------------------------------------------


//--------------------------------------------------------------------------
/**
 * @brief 将图像保存到指定文件
 * 
 * 该函数将图像保存到指定文件，图像格式根据文件名扩展名自动选择（支持格式见 cv::imread）。
 * 通常情况下，仅支持保存8位单通道或3通道（BGR通道顺序）图像，以下为例外情况：
 * 
 * - 16位无符号（CV_16U）图像可保存为PNG、JPEG 2000、TIFF格式；
 * - 32位浮点（CV_32F）图像可保存为PFM、TIFF、OpenEXR、Radiance HDR格式；
 *   3通道（CV_32FC3）TIFF图像会采用LogLuv高动态范围编码保存（每像素4字节）；
 * - 带透明通道的PNG图像可通过该函数保存：需创建8位/16位4通道BGRA图像（Alpha通道在最后），
 *   完全透明像素的Alpha值设为0，完全不透明像素的Alpha值设为255（8位）/65535（16位）；
 * - 多张图像（vector<Mat>）可保存为TIFF格式。
 * 
 * 若图像格式不被支持，会自动转换为8位无符号（CV_8U）格式后保存。
 * 若图像格式、深度或通道顺序不同，需先通过 Mat::convertTo 和 cv::cvtColor 转换，
 * 或使用通用的 FileStorage I/O 函数保存为XML/YAML格式。
 * 
 * 示例代码展示了BGRA图像创建、自定义压缩参数设置、PNG保存，以及多图像TIFF保存的方法：
 * @include snippets/imgcodecs_imwrite.cpp
 * 
 * @param filename [in] 保存文件的路径和名称（扩展名决定保存格式）
 * @param img [in] 待保存的图像（单张Mat或多张Mat的vector容器）
 * @param params [in] 格式特定的参数对（paramId_1, paramValue_1, paramId_2, paramValue_2...），
 *        取值参考 cv::ImwriteFlags，默认为空向量（使用默认参数）；常用参数如下：
 *        - IMWRITE_JPEG_QUALITY（JPG/JPEG）：0-100，0=最差/压缩率最高，100=最好/无压缩，默认95；
 *        - IMWRITE_PNG_COMPRESSION（PNG）：0-9，0=无压缩，9=最高压缩率，默认3；
 *        - IMWRITE_TIFF_COMPRESSION（TIFF）：1-9，1=无压缩、2=LZW、3=Deflate等；
 *        - IMWRITE_WEBP_QUALITY（WEBP）：0-100，0=最差，100=最好，默认100。
 * @return bool 保存成功返回true；保存失败返回false
 * 
 * @note 1. 彩色图像需保证为BGR通道顺序（OpenCV默认），RGB顺序保存会导致颜色错乱；
 *       2. 保存带透明通道的PNG时，必须使用BGRA通道顺序（Alpha通道最后），而非RGBA；
 *       3. 保存失败常见原因：路径不存在、权限不足、图像格式/深度不支持、文件名无有效扩展名；
 *       4. 保存多张图像到TIFF时，需将多张Mat放入vector<Mat>传入img参数，且所有Mat尺寸/通道数需一致；
 *       5. params参数需按「参数ID+参数值」成对传入，参数ID需与文件扩展名匹配，否则无效。
 * @sa cv::imread, cv::cvtColor, cv::Mat::convertTo, cv::FileStorage, cv::ImwriteFlags
 */
CV_EXPORTS_W bool imwrite( const String& filename, InputArray img, const std::vector<int>& params = std::vector<int>());
//--------------------------------------------------------------------------
  cv::imwrite("adjusted_image.jpg", params.dst_img, {IMWRITE_JPEG_QUALITY, 95});
//--------------------------------------------------------------------------


  cout << "处理后的图像已保存为 adjusted_image.jpg" << endl;


//--------------------------------------------------------------------------
/**
 * @brief 销毁所有已创建的HighGUI窗口
 * 
 * 该函数会关闭并释放程序中所有通过HighGUI创建的窗口资源（包括普通显示窗口、附带轨迹条的窗口），
 * 是程序退出前清理窗口资源的核心函数。
 * 
 * @note 1. 调用该函数后，所有窗口会立即关闭，对应的窗口名称和资源会被释放，无法再通过名称操作；
 *       2. 对于简单程序，操作系统会在程序退出时自动释放窗口资源，但显式调用该函数是更规范的做法；
 *       3. 若仅需销毁单个窗口，可使用 cv::destroyWindow 函数；
 *       4. 该函数无参数、无返回值，调用后无需额外校验，即使无已创建窗口也不会报错。
 * @sa cv::destroyWindow, cv::namedWindow, cv::imshow
 */
CV_EXPORTS_W void destroyAllWindows();
//--------------------------------------------------------------------------
  cv::destroyAllWindows();
//--------------------------------------------------------------------------


  return 0;
}
