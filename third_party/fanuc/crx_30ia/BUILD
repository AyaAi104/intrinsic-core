load("@rules_license//rules:license.bzl", "license")

package(
    default_applicable_licenses = [":license"],
    default_visibility = ["//visibility:public"],
)

license(
    name = "license",
    package_name = "fanuc",
    license_kinds = [
        "@rules_license//licenses/spdx:Apache-2.0",
    ],
    license_text = "LICENSE",
)

exports_files(["LICENSE"])

exports_files(["crx_30ia_base.xacro"])

filegroup(
    name = "crx_30ia_resources",
    srcs = [
        "LICENSE",
        "README.md",
    ] + glob(
        [
            "collision/*",
            "visual/*",
        ],
    ),
)
