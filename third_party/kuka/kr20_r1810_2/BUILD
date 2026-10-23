load("@rules_license//rules:license.bzl", "license")

package(
    default_applicable_licenses = [":license"],
    default_visibility = ["//visibility:public"],
)

license(
    name = "license",
    package_name = "kuka",
    license_kinds = [
        "@rules_license//licenses/spdx:Apache-2.0",
    ],
    license_text = "LICENSE",
)

exports_files(["LICENSE"])

filegroup(
    name = "kr20_r1810_2_resources",
    srcs = [
        "LICENSE",
        "README.md",
        "kr20_r1810_2_macro.xacro",
    ] + glob(
        [
            "collision/*",
            "visual/*",
        ],
    ),
)
