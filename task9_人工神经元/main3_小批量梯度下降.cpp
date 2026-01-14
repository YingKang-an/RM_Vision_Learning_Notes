#include <iostream>
#include <yaml-cpp/yaml.h>
#include <opencv2/core/core.hpp>
#include <random>
#include <algorithm>

// 初始化平面方程: 12x -34y -5z -9 = 0
// 平面方程:      1x - 2y + 3z - 4 = 0
// 目标平面参数:   A=1,B=-2,C=3,D=-4

int sign(float val) {
  return val >= 0 ? 1 : -1;
}

// 从YAML加载数据集到矩阵 
bool LoadDatasetFromYAML(const std::string& yaml_path, cv::Mat_<float>& X, cv::Mat_<int>& y) {
  YAML::Node _yaml;
    try {
      _yaml = YAML::LoadFile(yaml_path);
    } catch (const YAML::BadFile& e) {
      std::cerr << "[error] YAML文件加载失败:" << e.what() << std::endl;
      return false;
    }

  YAML::Node pos_pts = _yaml["dataset"]["positive_points"];
  YAML::Node neg_pts = _yaml["dataset"]["negative_points"];
  unsigned int total_num = pos_pts.size() + neg_pts.size();

  if (0 == total_num) {
    std::cerr << "[error] 数据集为空" << std::endl;
    return false;
  }

  // 初始化数据集矩阵
  X = cv::Mat_<float>(total_num, 4);  // n*4的特征矩阵
  y = cv::Mat_<int>(total_num, 1);    // n*1的标签矩阵

  int row_idx = 0;
    
  for (unsigned int i = 0; i < pos_pts.size(); ++i, ++row_idx) {
    X(row_idx, 0) = pos_pts[i]["x1"].as<float>();
    X(row_idx, 1) = pos_pts[i]["x2"].as<float>();
    X(row_idx, 2) = pos_pts[i]["x3"].as<float>();
    X(row_idx, 3) = 1.0f;

    y(row_idx, 0) = 1;
  }
    
  for (unsigned int i = 0; i < neg_pts.size(); ++i, ++row_idx) {
    X(row_idx, 0) = neg_pts[i]["x1"].as<float>();
    X(row_idx, 1) = neg_pts[i]["x2"].as<float>();
    X(row_idx, 2) = neg_pts[i]["x3"].as<float>();
    X(row_idx, 3) = 1.0f;

    y(row_idx, 0) = -1;
  }

  std::cout << "\n[Info] 数据集加载完成：" << std::endl;
  std::cout << "  总点数：" << total_num << "（正类：" << pos_pts.size() << "，负类：" << neg_pts.size() << "）" << std::endl;
  std::cout << "  目标平面方程: x - 2y + 3z - 4 = 0" << std::endl;
  
  return true;
}

// 行向量 * 列向量 = 对应元素乘积的`和`
//[x_n,y_n,z_n,1.0] * [A,B,C,D] = [x_n*A + y_n*B + z_n*C + D*1.0]

// 训练感知机（小批量梯度下降版本）
cv::Mat_<float> trainPerceptronMiniBatch(const cv::Mat_<float>& X, const cv::Mat_<int>& y, 
                                        float eta = 0.1f, int batch_size = 4, int max_iter = 1000) {
  // 初始化参数向量 theta：[A, B, C, D]^T
  cv::Mat_<float> theta = (cv::Mat_<float>(4, 1) << 12.0f, -34.0f, -5.0f, -9.0f);   // 初始参数

  // 迭代
  int iter = 0;
  int total_points = X.rows;
  
  // 创建索引数组用于随机打乱
  std::vector<int> indices(total_points);
  for (int i = 0; i < total_points; i++) {
    indices[i] = i;
  }
  
  // 创建随机数生成器
  std::random_device rd;
  std::mt19937 g(rd());
  
  std::cout << "=======================================================" << std::endl;
  std::cout << "目标平面: x - 2y + 3z - 4 = 0" << std::endl;
  std::cout << "学习率: " << eta << "，批量大小: " << batch_size << "，最大迭代次数: " << max_iter << std::endl;
  std::cout << "初始参数: A=" << theta(0,0) << ", B=" << theta(1,0) << ", C=" << theta(2,0) << ", D=" << theta(3,0) << std::endl;
  std::cout << "=======================================================" << std::endl;
  
  while (iter < max_iter) {
    // 使用随机数生成器打乱数据顺序（小批量梯度下降的关键）
    std::shuffle(indices.begin(), indices.end(), g);
    
    int total_misclassified = 0;
    int batch_count = 0;         // 每一批次里的样本数
    
    // 按批次处理数据
    for (int batch_start = 0; batch_start < total_points; batch_start += batch_size) {
      batch_count++;
      
      // 小批量梯度下降：收集当前批次的梯度
      cv::Mat_<float> batch_gradient = cv::Mat_<float>::zeros(4, 1);  // 批次梯度累加器
      int batch_misclassified = 0;
      
      // 处理当前批次
      for (int j = batch_start; j < std::min(batch_start + batch_size, total_points); j++) {
        int i = indices[j];  // 使用打乱后的索引
        
        // Score
        float S = X(i,0)*theta(0,0) + X(i,1)*theta(1,0) + X(i,2)*theta(2,0) + X(i,3)*theta(3,0);
        
        int _y = y(i, 0);       // y
        int _y_hat_ = sign(S);  // y_hat
        
        if (_y_hat_ != _y) {
          batch_misclassified++;
          total_misclassified++;
          // 小批量梯度下降：累加当前批次中误分类点的梯度
          cv::Mat_<float> x_vector = X.row(i).t();  // 将行向量转置为列向量
          batch_gradient += _y * x_vector;
        }
      }
      
      // 保存旧参数用于显示
      cv::Mat_<float> theta_old = theta.clone();
      
      // 小批量梯度下降：使用当前批次更新参数
      if (batch_misclassified > 0) {
        theta = theta + eta * batch_gradient;
        
        std::cout << "[第" << iter+1 << "轮] 批次" << batch_count << "更新|误分类点数:" << batch_misclassified << "\n"
                  << "A(" << theta_old(0, 0) << "→" << theta(0,0) << ")\n"
                  << "B(" << theta_old(1, 0) << "→" << theta(1,0) << ")\n"
                  << "C(" << theta_old(2, 0) << "→" << theta(2,0) << ")\n"
                  << "D(" << theta_old(3, 0) << "→" << theta(3,0) << ")\n" << std::endl;
      }
    }
    
    std::cout << "[第" << iter + 1 << "轮] 结束|总误分类点数:" << total_misclassified 
              << " | 当前参数: A=" << theta(0,0) << " B=" << theta(1,0)
              << " C=" << theta(2,0) << " D=" << theta(3,0) << std::endl;
    std::cout << "-------------------------------------------------------" << std::endl;
    
    if (total_misclassified == 0) {
      std::cout << "[Info] 感知机训练收敛！总迭代次数:" << iter + 1 << std::endl;
      break;
    }
    
    iter++;
  }
  
  if (iter >= max_iter) {
      std::cout << "[Info] 达到最大迭代次数，训练未完全收敛" << std::endl;
  }
  
  std::cout << "=======================================================" << std::endl;
  return theta;
}

// 验证感知机
bool validatePerceptron(const cv::Mat_<float>& X, const cv::Mat_<int>& y, 
                       const cv::Mat_<float>& theta) {
  int total_points = X.rows;
  int misclassified = 0;
  
  std::cout << "\n[validate] 开始验证，总点数: " << total_points << std::endl;
  std::cout << "学习到的平面方程: " << theta(0,0) << "x + " << theta(1,0) << "y + "
            << theta(2,0) << "z + " << theta(3,0) << " = 0" << std::endl;
  std::cout << "=======================================================" << std::endl;  
  
  for (int i = 0; i < total_points; ++i) {
    float S = X(i,0)*theta(0,0) + X(i,1)*theta(1,0) + X(i,2)*theta(2,0) + X(i,3)*theta(3,0);
      
    int _y_hat_ = sign(S);
    int _y = y(i, 0);
    
    if (_y_hat_ != _y) {
      misclassified++;
      std::cout << "[错误] 点" << i+1 << "(" << X(i,0) << "," << X(i,1) << "," << X(i,2) 
                << ") 预测:" << _y_hat_ << " 实际:" << _y << std::endl;
    } else {
      std::cout << "[正确] 点" << i+1 << "(" << X(i,0) << "," << X(i,1) << "," << X(i,2) 
                << ") 预测:" << _y_hat_ << " 实际:" << _y << std::endl;
    }
  }
  
  std::cout << "=======================================================" << std::endl;
  std::cout << "[validate] 完成！正确分类: " << (total_points - misclassified)
        << " | 误分类: " << misclassified << std::endl;
  
  return (misclassified == 0);
}

int main() {
  // 从YAML加载数据集
  cv::Mat_<float> X; // 特征矩阵
  cv::Mat_<int> y;   // 标签

  if (!LoadDatasetFromYAML("../dots.yaml", X, y)) {
    std::cerr << "程序退出：数据集加载失败" << std::endl;
    return -1;
  }
    
  // 训练感知机（使用小批量梯度下降）
  std::cout << "\n开始训练感知机（小批量梯度下降）..." << std::endl;
  
  // 使用小批量梯度下降版本
  // 参数：学习率=0.3，批量大小=4，最大迭代次数=10000
  cv::Mat_<float> theta = trainPerceptronMiniBatch(X, y, 0.1f, 5, 10000);
  
  // 验证结果
  bool success = validatePerceptron(X, y, theta);
  
  if (success) {
    std::cout << "\n========== 训练成功！ ==========" << std::endl;
    // 归一化比较（将平面方程A系数归一化为1）
    if (fabs(theta(0,0)) > 1e-6) {
      float scale = theta(0,0);
      std::cout << "\n目标平面方程A系数归一化为:" << std::endl;
      std::cout << "学习到的平面: x + " << theta(1,0)/scale << "y + "
                << theta(2,0)/scale << "z + " << theta(3,0)/scale << " = 0" << std::endl;
      std::cout << "目标平面:   x - 2y + 3z - 4 = 0" << std::endl;
      
      // 计算与目标平面的差异
      float diff_B = fabs(theta(1,0)/scale + 2.0f);
      float diff_C = fabs(theta(2,0)/scale - 3.0f);
      float diff_D = fabs(theta(3,0)/scale + 4.0f);
      
      std::cout << "\n与目标平面的差异:" << std::endl;
      std::cout << "B系数差异: " << diff_B << std::endl;
      std::cout << "C系数差异: " << diff_C << std::endl;
      std::cout << "D系数差异: " << diff_D << std::endl;
    }
  } else {
    std::cout << "\n========== 训练未完全收敛 ==========" << std::endl;
  }
  
  return 0;
}