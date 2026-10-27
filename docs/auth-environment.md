# Authentication endpoint configuration / 认证端点配置

Copy `.env.example` to `.env` in the directory from which you run the Go CLI. The supplied value preserves the existing public service address. The local `.env` is ignored by Git; do not commit credentials. / 将 `.env.example` 复制为运行 Go CLI 时当前目录下的 `.env`。示例值保持原公共服务地址。本地 `.env` 不纳入 Git，请勿提交凭据。

```dotenv
INTRINSIC_TOKEN_EXCHANGE_SERVER=flowstate.intrinsic.ai
```

Resolution order / 优先级：

1. Explicit `WithTokenExchangeServiceAddress(...)` option, including an empty value. / 显式 API 选项，包括空值。
2. Process environment `INTRINSIC_TOKEN_EXCHANGE_SERVER`, including an empty value. / 进程环境变量，包括空值。
3. File selected by `INTRINSIC_ENV_FILE`, or `.env` in the current directory. / 指定配置文件，未指定时使用当前目录 `.env`。
4. Packaged `defaults.env` if the optional file or key is absent. / 缺少可选文件或键时使用打包默认值。

Linux example / Linux 示例：

```sh
export INTRINSIC_ENV_FILE=/absolute/path/to/.env
inctl auth list
```

The address is consumed when Go authentication converts API-key credentials to ID-token credentials; commands not using that conversion need no dotenv access. Configurations do not propagate automatically to containers, Python clients, or other processes. / 地址在 Go 认证模块将 API key 凭据转换为 ID token 凭据时读取；不使用该转换的命令无需访问 dotenv。配置不会自动传入容器、Python 客户端或其他进程。

The resolver uses the already pinned `godotenv` package to read values without mutating process-wide environment variables. Missing explicitly selected files and malformed/unreadable files return configuration errors unless an explicit API/environment address bypasses file loading. With no override or file, the original default remains available in installed binaries and Bazel sandboxes. / 解析器使用已锁定的 godotenv 读取配置，不修改全进程环境变量。显式指定文件不存在或配置无法解析/读取时返回错误；显式 API/环境地址会跳过文件读取。无覆盖或文件时，安装后的二进制及 Bazel 沙箱仍保留原默认值。

Python syntax keywords and empty pagination cursors are not environment settings. / Python 语法关键字和空分页游标不属于环境配置。

Verification / 验证： `bazel test //intrinsic_sdk/intrinsic/tools/inctl/auth:config_test` on the supported build environment. Configuration helper tests were also run in an isolated module against the exact source and pinned dependency. / 在支持的构建环境执行 Bazel 测试；配置辅助函数也已用原样源码和锁定依赖在独立模块中测试。
