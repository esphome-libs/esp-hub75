# SPDX-FileCopyrightText: 2026 Stuart Parmenter
# SPDX-License-Identifier: MIT

idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "." "../../../common"
    REQUIRES hub75
)
