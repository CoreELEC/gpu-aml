# SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
#
# (C) COPYRIGHT 2025 ARM Limited. All rights reserved.
#
# This program is free software and is provided to you under the terms of the
# GNU General Public License version 2 as published by the Free Software
# Foundation, and any use by you of this program is subject to the terms
# of such GNU license.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, you can access it online at
# http://www.gnu.org/licenses/gpl-2.0.html.
#
#

KUTF_CFLAGS = [
    "-Wno-sign-compare",
    "-Wno-unused-but-set-variable",
    "-Wno-unused-parameter",
]

CFLAGS_MODULE = [
    "-Wall",
    "-Werror",
    "-Wextra",
    "-Wunused",
    "-Wno-unused-parameter",
    "-Wmissing-declarations",
    "-Wmissing-prototypes",
    "-Wold-style-definition",
    "-Wunused-const-variable",
    "-Wno-sign-compare",
    "-Wno-shift-negative-value",
    "-Wno-cast-function-type",
    "-Wframe-larger-than=4096",
    "-Wdisabled-optimization",
    "-Wmissing-field-initializers",
    "-Wno-type-limits",
    "-Wunused-macros",
] + select({
    "//config:cov_kernel": [
        "-DGCOV_PROFILE=1",
        "-ftest-coverage",
        "-fprofile-arcs",
    ],
    "//conditions:default": [],
}) + select({
    "//config:mali_kcov": [
        "-DKCOV=1",
        "-DKCOV_ENABLE_COMPARISONS=1",
        "-fsanitize-coverage=trace-cmp",
    ],
    "//conditions:default": [],
})

COPTS_KBASE = [
    "-DMALI_COVERAGE=0",
    "-DMALI_JIT_PRESSURE_LIMIT_BASE=1",
    "-DMALI_USE_CSF=0",
    "-DMALI_RELEASE_NAME=\"r54p1-12eac0\"",
] + select({
    "//config:mali_debug": ["-DMALI_UNIT_TEST=1"],
    "//conditions:default": ["-DMALI_UNIT_TEST=0"],
}) + select({
    "//config:mali_has_cl": ["-DMALI_HAS_CL=1"],
    "//conditions:default": ["-DMALI_HAS_CL=0"],
}) + select({
    "//config:mali_has_gles": ["-DMALI_HAS_GLES=1"],
    "//conditions:default": ["-DMALI_HAS_GLES=0"],
}) + select({
    "//config:mali_customer_release": ["-DMALI_CUSTOMER_RELEASE=0"],
    "//conditions:default": ["-DMALI_CUSTOMER_RELEASE=0"],
}) + select({
    "//config:mali_expert_debug": ["-DMALI_KERNEL_TEST_API=1"],
    "//conditions:default": ["-DMALI_KERNEL_TEST_API=0"],
})
