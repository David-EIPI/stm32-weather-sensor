from zigpy.quirks.v2 import QuirkBuilder
from zigpy.quirks.v2.homeassistant import EntityType, UnitOfPressure
from zigpy.quirks.v2.homeassistant.sensor import SensorDeviceClass, SensorStateClass
from zigpy.zcl.clusters.measurement import PressureMeasurement
from zha.application.platforms.sensor import ElectricalMeasurementRMSVoltage

from zha.zigbee.cluster_handlers.measurement import (
        RelativeHumidityClusterHandler,
        TemperatureMeasurementClusterHandler,
)

from zha.zigbee.cluster_handlers.const import REPORT_CONFIG_DEFAULT, REPORT_CONFIG_MIN_INT, REPORT_CONFIG_MAX_INT
from zha.zigbee.cluster_handlers import AttrReportConfig

from zigpy.zcl.clusters.measurement import (
        RelativeHumidity,
        TemperatureMeasurement,
)

# configure smaller change threshold for reports (default = 100)
RelativeHumidityClusterHandler.REPORT_CONFIG = (
        AttrReportConfig(
            attr=RelativeHumidity.AttributeDefs.measured_value.name,
            config=(REPORT_CONFIG_MIN_INT, REPORT_CONFIG_MAX_INT, 10),
        ),
    )

TemperatureMeasurementClusterHandler.REPORT_CONFIG = (
        AttrReportConfig(
            attr=TemperatureMeasurement.AttributeDefs.measured_value.name,
            config=(REPORT_CONFIG_MIN_INT, REPORT_CONFIG_MAX_INT, 10),
        ),
)

import zigpy.types as t

# increase display accuracy for electrical measurements (default = 1)
ElectricalMeasurementRMSVoltage._decimals = 3

# add support for scaled pressure attribute with higher precision
(
    QuirkBuilder("DS", "Weather1")
    .sensor(
        "scaled_value",
        PressureMeasurement.cluster_id,
        divisor = 10,
        unit = UnitOfPressure.HPA,
        entity_type = EntityType.STANDARD,
        fallback_name = "Athmosperic pressure",
        device_class = SensorDeviceClass.ATMOSPHERIC_PRESSURE,
        state_class = SensorStateClass.MEASUREMENT,
    )
    .add_to_registry()
)

