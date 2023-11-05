# Copyright (c) 2023 UltiMaker
# CuraEngineTiledInfill is released under the terms of the LGPLv3 or higher.

import os
import platform
import stat
import sys
from pathlib import Path
from typing import List, Tuple

from UM.Logger import Logger
from UM.Settings.ContainerRegistry import ContainerRegistry
from UM.Settings.DefinitionContainer import DefinitionContainer
from UM.i18n import i18nCatalog
from cura.BackendPlugin import BackendPlugin

catalog = i18nCatalog("cura")


class CuraEnginePattern(BackendPlugin):
    def __init__(self):
        super().__init__()
        self.definition_file_paths = [Path(__file__).parent.joinpath("infill_settings.def.json").as_posix()]
        self._tiles_path = Path(__file__).parent.joinpath("tiles")
        if not self.isDebug():
            if not self.binaryPath().exists():
                Logger.error(f"Could not find CuraEnginePattern binary at {self.binaryPath().as_posix()}")
            if platform.system() != "Windows" and self.binaryPath().exists():
                st = os.stat(self.binaryPath())
                os.chmod(self.binaryPath(), st.st_mode | stat.S_IEXEC)

            self._plugin_command = [self.binaryPath().as_posix()]

        self._supported_slots = [200]  # Infill Generation SlotID
        ContainerRegistry.getInstance().containerLoadComplete.connect(self._on_container_load_complete)

    def _on_container_load_complete(self, container_id) -> None:
        if not ContainerRegistry.getInstance().isLoaded(container_id):
            # skip containers that could not be loaded, or subsequent findContainers() will cause an infinite loop
            return
        try:
            container = ContainerRegistry.getInstance().findContainers(id=container_id)[0]
        except IndexError:
            # the container no longer exists
            return
        if not isinstance(container, DefinitionContainer):
            # skip containers that are not definitions
            return
        if container.getMetaDataEntry("type") == "extruder":
            # skip extruder definitions
            return

        for pattern_key in ["infill_pattern"]:  #, "support_pattern"]:
            for definition in container.findDefinitions(key=pattern_key):
                for pattern in self.getPatterns():
                    definition.extend_category(pattern[0], pattern[1], plugin_id=self.getPluginId(), plugin_version=self.getVersion())

        for pattern_key in ["infill_pattern",]:  #"top_bottom_pattern", "ironing_pattern",  "support_pattern", "support_interface_pattern", "support_roof_pattern", "support_bottom_pattern", "roofing_pattern"]:
            for definition in container.findDefinitions(key=pattern_key):
                for pattern in self.getSpaceFillingPatterns():
                    definition.extend_category(pattern[0], pattern[1], plugin_id=self.getPluginId(), plugin_version=self.getVersion())

        for definition in container.findDefinitions(key="connect_infill_polygons"):
            x = 1

    def getPatterns(self) -> List[Tuple[str, str]]:
        return []

    def getSpaceFillingPatterns(self) -> List[Tuple[str, str]]:
        return [("HILBERT", "Hilbert Curve")]

    def getPort(self):
        return int(os.environ["CURAENGINE_PATTERN_PORT"])
        # return super().getPort() if not self.isDebug() else int(os.environ["CURAENGINE_PATTERN_PORT"])

    def isDebug(self):
        return os.environ.get("CURAENGINE_PATTERN_PORT", None) is not None
        # return not hasattr(sys, "frozen") and os.environ.get("CURAENGINE_PATTERN_PORT", None) is not None

    def start(self):
        if not self.isDebug():
            super().start()

    def binaryPath(self) -> Path:
        ext = ".exe" if platform.system() == "Windows" else ""

        machine = platform.machine()
        if machine == "AMD64":
            machine = "x86_64"
        return Path(__file__).parent.joinpath(machine, platform.system(), f"curaengine_plugin_pattern{ext}").resolve()
