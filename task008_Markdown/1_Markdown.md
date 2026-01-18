# <center><font face="黑体" font color=orange > Markdown教程</font></center>
## <center><font face="黑体" size=5 font color=cyan>YinKang'an</font></center>

### 一.准备工作

1. **安装vscode**
     >[vscode下载地址]  http://code.visualstudio.com/
2. **下载必要插件**
    - Markdown All in One
    - Markdown Preview in hanced
    - Markdown PDF (但不是很推荐)
    - Markdown Image
3. **创建.md文档，打开同步预览功能，开始编辑**

### 二.基本语法

1. **标题**
    #一级文档
    ##二级文档
    ...最多支持6级标题
2. **引用**
    >一级引用
    >>二级引用
3. **列表**
    1. 无序列表
       - 列表1
       + 列表二
       * 列表三( - + * )
    2. 有序列表( <-like this ) 
    3. 列表嵌套(1.2.3/i,ii,iii/...)
    4. TodoList
        - [x] a
        - [ ] b
        - [ ] c
4. **表格**
    | 左对齐 | 居中 | 右对齐 |
    |:- |:-:|-:|
    | a | b | c |
5. **段落**
    - 换行？---- 两个以上空格后回车/空一行
        [换行]
    - 分割线？---- 三个*
        ***
    - 字体
        | 字体 | 代码 |
        |:-:|:-:|
        |*斜体*|* *|
        |==高亮==|== ==|
        |**粗体**|** **|
        |***斜粗体***|*** ***|
        |~~删除~~|~~ ~~|
        |<u>下划线</u>|```<u> </u>```|
    - 脚注
        这是一个脚注[^1]哦
        [^1]:脚注在这注解一下
6. **代码**
    - 单句代码
    ` print"hello!";`
    - 代码块
    ```cpp
    class Test         //class 关键字， Test 类名，一般首字母大写
    {
    private:           //私有访问权限
        int a = 12;
    public:            //公共访问权限
        void Print()            //输出数据
        {
            cout << a << endl;
        }
        int Get()               //外界获取数据
        {
            return a;
        }
    };
    ```
7. **超链接**
    - [更多教程可参考网站] : https://www.runoob.com/markdown/md-link.html
    - 点击连接的文字-> [点击连接][教程]

[教程]:https://www.runoob.com/markdown/md-link.html

8. **图片**
    - 使用图床保存图片并插入
        免费图床:[路过图床][路过图床]

    [路过图床]:https://imgse.com/

    - 使用Markdown语法插入
    [![pZNOn0J.md.jpg](https://s41.ax1x.com/2026/01/01/pZNOn0J.md.jpg)](https://imgchr.com/i/pZNOn0J)

    - 使用HTML语言调整图片位置和大小
    <a href="https://imgchr.com/i/pZNOn0J"><div align=center><img src="https://s41.ax1x.com/2026/01/01/pZNOn0J.jpg" alt="pZNOn0J.jpg" border="0" width=50% height=50%/></div></a>

### 三.其他操作
   - **插入LaTeX数学公式**
        - 行内显示公式:
            $f(x)=ax+b$
        - 块内显示数学表达式:
            $$
            \begin{Bmatrix}
            a & b \\
            c & d
            \end{Bmatrix}
            $$
   - **htlm/css语法**
        - ctrl+shift+p 搜索
        "Markdown Preview Enhanced:Customize CSS"
        - 在style中使用css语法改格式
   - 个性化操作
        - File-Preferense-Settings
### 四.导出为PDF
- Expore-HTML-offline-浏览器打开-导出PDF