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
    name = "kr10_r1100_2_resources",
    srcs = [
        "LICENSE",
        "README.md",
        "kr10_r1100_2_macro.xacro",
    ] + glob(
        [
            "collision/*",
            "visual/*",
        ],
    ),
)
