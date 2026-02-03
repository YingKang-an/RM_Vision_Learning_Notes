#!/bin/bash

# 张正友标定程序一键启动脚本
# 作者: YinKang'an
# 日期: 2026-02-02
# 版本: 1.0.0

# 设置颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 项目根目录
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="$PROJECT_DIR/build"
EXECUTABLE="$BUILD_DIR/main"

# 函数：打印带颜色的消息
print_info() {
  echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
  echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
  echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
  echo -e "${RED}[ERROR]${NC} $1"
}

# 函数：检查命令是否存在
check_command() {
  if ! command -v $1 &> /dev/null; then
    print_error "命令 '$1' 未安装，请先安装"
    return 1
  fi
  return 0
}

# 函数：检查依赖项
check_dependencies() {
  print_info "检查系统依赖项..."
  
  # 检查CMake
  if ! check_command "cmake"; then
    print_error "请安装CMake: sudo apt install cmake"
    return 1
  fi
  
  # 检查OpenCV
  if ! pkg-config --exists opencv4; then
    print_error "OpenCV未正确安装"
    return 1
  fi
  
  # 检查yaml-cpp
  if ! pkg-config --exists yaml-cpp; then
    print_error "yaml-cpp未正确安装"
    return 1
  fi
  
  # 检查大恒相机SDK
  if [ ! -d "/opt/Galaxy_camera" ]; then
    print_error "大恒相机SDK未安装或路径不正确"
    print_warning "请确保大恒相机SDK安装在 /opt/Galaxy_camera"
    return 1
  fi
  
  print_success "所有依赖项检查通过"
  return 0
}

# 函数：清理构建目录
clean_build() {
  print_info "清理构建目录..."
  if [ -d "$BUILD_DIR" ]; then
    rm -rf "$BUILD_DIR"
    print_success "构建目录已清理"
  else
    print_warning "构建目录不存在，跳过清理"
  fi
}

# 函数：编译项目
compile_project() {
  print_info "开始编译项目..."
  
  # 创建构建目录
  mkdir -p "$BUILD_DIR"
  cd "$BUILD_DIR"
  
  # 运行CMake配置
  print_info "运行CMake配置..."
  if ! cmake ..; then
    print_error "CMake配置失败"
    return 1
  fi
  
  # 编译项目
  print_info "编译项目..."
  if ! make -j$(nproc); then
    print_error "编译失败"
    return 1
  fi
  
  print_success "项目编译完成"
  return 0
}

# 函数：检查配置文件
check_config() {
  print_info "检查配置文件..."
  
  local config_file="$PROJECT_DIR/config/config.yaml"
  local version_file="$PROJECT_DIR/config/version.yaml"
  
  if [ ! -f "$config_file" ]; then
    print_error "配置文件不存在: $config_file"
    return 1
  fi
  
  if [ ! -f "$version_file" ]; then
    print_warning "版本文件不存在: $version_file"
  fi
  
  print_success "配置文件检查完成"
  return 0
}

# 函数：运行程序
run_program() {
  print_info "启动标定程序..."
  
  if [ ! -f "$EXECUTABLE" ]; then
    print_error "可执行文件不存在: $EXECUTABLE"
    return 1
  fi
  
  # 切换到build目录运行程序，确保相对路径正确
  cd "$BUILD_DIR"
  
  print_success "程序启动成功！"
  echo "=========================================="
  echo "使用说明:"
  echo "- SPACE键:  采集标定图像"
  echo "- C键:      开始标定计算"
  echo "- S键:      保存标定结果"
  echo "- ESC键:    退出程序"
  echo "=========================================="
  
  # 运行程序
  ./main
  
  local exit_code=$?
  if [ $exit_code -eq 0 ]; then
    print_success "程序正常退出"
  else
    print_error "程序异常退出，退出码: $exit_code"
  fi
  
  return $exit_code
}

# 函数：显示帮助信息
show_help() {
  echo "张正友标定程序一键启动脚本"
  echo ""
  echo "用法: $0 [选项]"
  echo ""
  echo "选项:"
  echo "  -c, --clean     清理构建目录后重新编译"
  echo "  -r, --run-only  仅运行程序（不重新编译）"
  echo "  -h, --help      显示此帮助信息"
  echo "  -v, --version   显示版本信息"
  echo ""
  echo "示例:"
  echo "  $0              # 正常编译并运行"
  echo "  $0 -c           # 清理后重新编译运行"
  echo "  $0 -r           # 仅运行已编译的程序"
}

# 函数：显示版本信息
show_version() {
  local version_file="$PROJECT_DIR/config/version.yaml"
  if [ -f "$version_file" ]; then
    echo "版本信息:"
    cat "$version_file"
  else
    echo "版本: 1.0.0"
    echo "作者: YinKang'an"
    echo "日期: 2026-02-01"
  fi
}

# 主函数
main() {
  local clean_build_flag=false
  local run_only_flag=false
  
  # 解析命令行参数
  while [[ $# -gt 0 ]]; do
    case $1 in
      -c|--clean)
        clean_build_flag=true
        shift
        ;;
      -r|--run-only)
        run_only_flag=true
        shift
        ;;
      -h|--help)
        show_help
        exit 0
        ;;
      -v|--version)
        show_version
        exit 0
        ;;
      *)
        print_error "未知参数: $1"
        show_help
        exit 1
        ;;
    esac
  done
  
  # 显示启动信息
  echo "=========================================="
  echo "        张正友标定程序一键启动脚本"
  echo "=========================================="
  
  # 检查依赖项
  if ! check_dependencies; then
    print_error "依赖项检查失败，请先安装必要的依赖"
    exit 1
  fi
  
  # 检查配置文件
  if ! check_config; then
    print_error "配置文件检查失败"
    exit 1
  fi
  
  # 清理构建目录（如果指定了-c参数）
  if [ "$clean_build_flag" = true ]; then
    clean_build
  fi
  
  # 编译项目（如果不是仅运行模式）
  if [ "$run_only_flag" = false ]; then
    if ! compile_project; then
      print_error "编译失败，程序退出"
      exit 1
    fi
  else
    print_info "跳过编译，直接运行程序"
  fi
  
  # 运行程序
  if ! run_program; then
    print_error "程序运行失败"
    exit 1
  fi
  
  print_success "start.sh 执行完毕!"
}

# 脚本入口点
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  main "$@"
fi