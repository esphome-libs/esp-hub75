"""Keep push/PR builds small; exercise all SDK/memory variants weekly or manually."""

import json
import os

full = os.environ["GITHUB_EVENT_NAME"] in ("schedule", "workflow_dispatch")
versions = ["v4.4.8", "v5.5.5", "v6.1"] if full else ["v6.1"]
builds = []

for version in versions:
    for target in ("esp32", "esp32s2", "esp32s3", "esp32c6", "esp32p4"):
        if version == "v4.4.8" and target in ("esp32c6", "esp32p4"):
            continue

        framebuffers = ["default"]
        if target == "esp32s3":
            framebuffers.append("psram")
        elif target == "esp32p4":
            framebuffers = ["default", "psram", "internal"] if full else ["psram"]

        for framebuffer in framebuffers:
            builds.append({"idf_target": target, "idf_ver": version, "framebuffer": framebuffer})

print(json.dumps({"include": builds}))
