# GitHub 推送避坑（方舟 / omnibot-f4）

以后只要用户说“推 GitHub / 提交 / 开 PR”，默认按这份说明做，不要先在外层仓库瞎转。

## 1. 真正要推的仓库

| 路径 | 是什么 | 是否推 |
|------|--------|--------|
| `D:\project\fangzhou\-` | 固件仓库，remote = `https://github.com/tian11111/omnibot-f4` | **推这个** |
| `D:\project\fangzhou` | 外层杂项仓库，**没有 remote / 还没有效提交** | 不要当主仓库推 |
| `D:\project\fangzhou\AndroidApp` | 另一个仓库 `tian11111/bt-controller` | 只有用户明确说 Android 才推 |

一句话：

```text
固件改动 -> cd D:\project\fangzhou\-
```

## 2. 网络代理坑

本机环境常有坏代理：

```text
GIT_HTTP_PROXY=http://127.0.0.1:9
GIT_HTTPS_PROXY=http://127.0.0.1:9
```

这会导致 `git push` / `gh api` 卡住或失败。

推送前先清空：

```powershell
$env:HTTPS_PROXY=''
$env:HTTP_PROXY=''
$env:ALL_PROXY=''
$env:GIT_HTTP_PROXY=''
$env:GIT_HTTPS_PROXY=''
```

`gh` 如果报 keyring token 无效，可先：

```powershell
$env:HTTPS_PROXY=''; $env:HTTP_PROXY=''; $env:ALL_PROXY=''; $env:GIT_HTTP_PROXY=''; $env:GIT_HTTPS_PROXY=''
gh auth status
```

很多时候清代理后就能用。

## 3. 只提交源码，不提交编译垃圾

固件仓库默认只 stage 这些相关源文件（按实际改动增减）：

```text
Core/Inc/*.h
Core/Src/*.c
```

**不要**默认提交：

```text
MDK-ARM/*.o *.d *.axf *.hex *.map *.htm *.lnp
MDK-ARM/*.uvguix.* *.uvoptx *.dep
*.ioc 临时（除非用户明确要求）
.vs/ _backups/ temp_* 截图 PDF 手册
```

## 4. 推荐最短流程

```powershell
cd D:\project\fangzhou\-

# 1) 清代理
$env:HTTPS_PROXY=''; $env:HTTP_PROXY=''; $env:ALL_PROXY=''; $env:GIT_HTTP_PROXY=''; $env:GIT_HTTPS_PROXY=''

# 2) 看改动
git status -sb
git diff --stat

# 3) 建分支
git checkout -b codex/<short-desc>

# 4) 只加源码
git add Core/Inc/... Core/Src/...

# 5) 提交
git commit -m "..."

# 6) 推送
git push -u origin HEAD

# 7) 开 draft PR
gh pr create --repo tian11111/omnibot-f4 --base main --head (git branch --show-current) --draft --title "..." --body "..."
```

## 5. 这次慢的原因（复盘）

1. 先看了外层 `D:\project\fangzhou`，它只是空壳/杂文件暂存，不是固件 remote。  
2. 被 `127.0.0.1:9` 代理坑了，网络请求失败/卡住。  
3. 工作区混有大量 Keil 编译产物，需要先筛源码再提交。  
4. 还额外确认了 Android 子仓、gh 登录、draft PR，步骤偏多。

下次默认：

1. 直接进 `D:\project\fangzhou\-`  
2. 先清代理  
3. 只提交 Core 源码  
4. push + draft PR  

不要再从外层仓库开始排查。