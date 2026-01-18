# <center><font face="黑体" font color=orange > LaTeX语法</font></center>
## <center><font face="黑体" size=5 font color=cyan>YinKang'an</font></center>

## 希腊字母
用`\`加上英文拼写
>1.数学公式要包在`$$`里面，
2.`\\`换行, 
3.大写就把`首字母`大写,
4.`var`变体

| 小写希腊字母       | 大写希腊字母       | 字母变体           |
|--------------------|--------------------|--------------------|
| $\alpha$：`\alpha` | $\Alpha$：`\Alpha` | $\varGamma$：`\varGamma` |
| $\beta$：`\beta`   | $\Beta$：`\Beta`   | $\varDelta$：`\varDelta` |
| $\gamma$：`\gamma` | $\Gamma$：`\Gamma` | $\varTheta$：`\varTheta` |
| $\delta$：`\delta` | $\Delta$：`\Delta` | $\varLambda$：`\varLambda` |
| $\epsilon$：`\epsilon` | $\Epsilon$：`\Epsilon` | $\varXi$：`\varXi` |
| $\zeta$：`\zeta`   | $\Zeta$：`\Zeta`   | $\varPi$：`\varPi` |
| $\eta$：`\eta`     | $\Eta$：`\Eta`     | $\varSigma$：`\varSigma` |
| $\theta$：`\theta` | $\Theta$：`\Theta` | $\varPhi$：`\varPhi` |
| $\iota$：`\iota`   | $\Iota$：`\Iota`   | $\varPsi$：`\varPsi` |
| $\kappa$：`\kappa` | $\Kappa$：`\Kappa` | $\varOmega$：`\varOmega` |
| $\lambda$：`\lambda` | $\Lambda$：`\Lambda` | $\vartheta$：`\vartheta` |
| $\mu$：`\mu`       | $\Mu$：`\Mu`       | $\varpi$：`\varpi` |
| $\nu$：`\nu`       | $\Nu$：`\Nu`       | $\varrho$：`\varrho` |
| $\xi$：`\xi`       | $\Xi$：`\Xi`       | $\varsigma$：`\varsigma` |
| $\omicron$：`\omicron` | $\Omicron$：`\Omicron` | $\varepsilon$：`\varepsilon` |
| $\pi$：`\pi`       | $\Pi$：`\Pi`       | $\phi$：`\phi` |
| $\rho$：`\rho`     | $\Rho$：`\Rho`     | $\varphi$：`\varphi` |
| $\sigma$：`\sigma` | $\Sigma$：`\Sigma` |                    |
| $\tau$：`\tau`     | $\Tau$：`\Tau`     |                    |
| $\upsilon$：`\upsilon` | $\Upsilon$：`\Upsilon` |                    |
| $\chi$：`\chi`     | $\Chi$：`\Chi`     |                    |
| $\psi$：`\psi`     | $\Psi$：`\Psi`     |                    |
| $\omega$：`\omega` | $\Omega$：`\Omega` |                    |

## 上下标
>1.用`^`和`_`表示,
2.各种运算符和命令产生的格式效果都只
  对其后面大括号内的各字符有效
  (若其内只有一个字符则可省略大括号)

$$
a^2,a_1 \\
x^{y+z},P_{ij},
$$

>斜体下表表示变量 $X_i$
直立体下表表示输入输出常量 $X_{\rm i}$ ,或 $X_{\text i}$
其中`{\rm i}`中`\rm`表示`roman`罗马体

>$\text A B$,$\rm A B$
`{\text}`支持空格,但只会把跟随他的字母直立
`\rm`不支持空格,但后面所有字母直立

## 分式 与 根式
>1.写法:分式`\frac{分子}{分母}`,根式:`\sqrt`
 2.`\dfrac`：变大一点

$$
\frac{1}{2},\frac 1 2,\frac 1 {x+y},\frac{\dfrac 1 x + 1}{y + 1} \\
$$
$$
\sqrt 2,\sqrt{x+y},\sqrt[3]{x+y}
$$

## 普通运算符

$$
\times,\cdot,\div,+ ,- \\
\pm,\mp\\
\lt,\gt,\ge,\le,><,\gg,\ll,\ne,\approx,\equiv\\
\cap,\cup,\in,\notin,\subseteq,\subsetneq,\subsetneqq,\varnothing\\
\forall,\exists,\nexists\\
\because,\therefore\\
\R,\mathbb Q,\N,\Z,\Z_+\\
\mathcal F,\mathscr F\\
\cdots,\vdots,\ddots\\
\infty,\partial,\nabla,\propto,\degree\\
\sin x,\sec x,\cosh x\\
\log_2 x,\ln x,\lg x\\
\lim_{x \to 0} \frac{x}{sin x}\\
\max \limits x,\min x\\
$$
