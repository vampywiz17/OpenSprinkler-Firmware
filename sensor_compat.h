/* OpenSprinkler Unified (AVR/RPI/BBB/LINUX) Firmware
 * Copyright (C) 2015 by Ray Wang (ray@opensprinkler.com)
 * Analog Sensor API by Stefan Schmaltz (info@opensprinklershop.de)
 *
 * Upstream "Expanded Sensor" API compatibility layer
 * Sep 2026 @ OpenSprinklerShop
 *
 * Official OpenSprinkler firmware 2.2.1(5) introduced an "Expanded Sensor"
 * API (/jsn, /csn, /dsn, /jsd, /jsl, /dsl, /jpa, program "snadj"). This
 * module exposes the OpenSprinklerShop sensor subsystem (sensors.h,
 * SensorBase.hpp) through that API so that the official OpenSprinkler App
 * and third-party clients written against the official API work unchanged.
 *
 * It is a facade only: there is a single sensor store (sensors.json,
 * progsensor.json, sensorlog*.dat). Upstream "uuid" == our sensor "nr".
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#ifndef _SENSOR_COMPAT_H
#define _SENSOR_COMPAT_H

#include <stdint.h>
#include "sensors.h"

class SensorBase;
class ProgSensorAdjust;
class BufferFiller;

// ---------------------------------------------------------------------------
// Upstream enumerations. Numeric values are part of the public API contract
// (they are the values transported in /jsn, /csn and /jsd) and MUST stay in
// sync with the official firmware (sensors/sensor.h upstream).
// ---------------------------------------------------------------------------

// Index into /jsd.sensors. 0..4 are the official types; 5 is our extension
// that exposes every other OpenSprinklerShop sensor type (RS485, MQTT,
// Zigbee, BLE, FYTA, ...) generically.
enum class CompatSensorType : uint8_t {
	Aggregate = 0,
	ADS1115 = 1,
	Weather = 2,
	SystemInternal = 3,
	OnboardDigital = 4,
	Native = 5,
	MAX_VALUE,
};

// X(id, display_name)
#define COMPAT_AGGREGATE_ACTION_LIST(X) \
	X(Min,     "Min")     \
	X(Max,     "Max")     \
	X(Average, "Average") \
	X(Sum,     "Sum")     \
	X(Median,  "Median")  \
	X(Range,   "Range")

enum class CompatAggregateAction : uint8_t {
#define X(id, name) id,
	COMPAT_AGGREGATE_ACTION_LIST(X)
#undef X
	MAX_VALUE,
};

// Upstream declares WeatherAction empty ("disabled this release"). We
// implement it with the weather values our SENSOR_WEATHER_* types provide.
// X(id, display_name, native_sensor_type)
#define COMPAT_WEATHER_ACTION_LIST(X) \
	X(TemperatureF, "Temperature (°F)",  SENSOR_WEATHER_TEMP_F)    \
	X(TemperatureC, "Temperature (°C)",  SENSOR_WEATHER_TEMP_C)    \
	X(Humidity,     "Humidity (%)",      SENSOR_WEATHER_HUM)       \
	X(PrecipIn,     "Precipitation (in)",SENSOR_WEATHER_PRECIP_IN) \
	X(PrecipMm,     "Precipitation (mm)",SENSOR_WEATHER_PRECIP_MM) \
	X(WindMph,      "Wind (mph)",        SENSOR_WEATHER_WIND_MPH)  \
	X(WindKmh,      "Wind (km/h)",       SENSOR_WEATHER_WIND_KMH)  \
	X(Eto,          "ETo",               SENSOR_WEATHER_ETO)       \
	X(Radiation,    "Solar radiation",   SENSOR_WEATHER_RADIATION)

enum class CompatWeatherAction : uint8_t {
#define X(id, name, ntype) id,
	COMPAT_WEATHER_ACTION_LIST(X)
#undef X
	MAX_VALUE,
};

// Upstream SystemMetric values (stable across platforms)
enum class CompatSystemMetric : uint8_t {
	FREE_HEAP          = 0,  // KB
	FREE_FLASH         = 1,  // KB
	WIFI_RSSI          = 2,  // dBm (not provided by us)
	CPU_TEMPERATURE    = 3,  // °C
	HEAP_FRAGMENTATION = 4,  // % (not provided by us)
	MAX_VALUE,
};

// Upstream ADS1115Subtype values
enum class CompatAds1115Subtype : uint8_t {
	LINEAR               = 0,
	PIECEWISE_LINEAR     = 1,
	SMT50_TEMPERATURE    = 10,
	SMT50_MOISTURE       = 11,
	SMT100_TEMPERATURE   = 12,
	SMT100_MOISTURE      = 13,
	SMT100_PERMITTIVITY  = 14,
	VH400_MOISTURE       = 15,
	THERM200_TEMPERATURE = 16,
	AQUAPLUMB_LEVEL      = 17,
	MAX_VALUE,
};

// Upstream OnboardInput values
enum class CompatOnboardInput : uint8_t {
	SN1 = 0,
	SN2 = 1,
	SN3 = 2,
	SN4 = 3,
	MAX_VALUE,
};

// X(id, display_name) — verbatim upstream
#define COMPAT_UNIT_GROUP_LIST(X) \
	X(None,        "No Unit")    \
	X(Energy,      "Energy")      \
	X(Flow,        "Flow")        \
	X(Pressure,    "Pressure")    \
	X(Temperature, "Temperature") \
	X(Light,       "Light")       \
	X(Length,      "Length")      \
	X(Velocity,    "Velocity")    \
	X(Volume,      "Volume")      \
	X(Salinity,    "Salinity")    \
	X(Angle,       "Angle")       \
	X(Precipitation, "Precipitation")

// X(id, display_name, short_symbol, group_id) — verbatim upstream; the
// enumerator index is the "unit" value on the wire.
#define COMPAT_UNIT_LIST(X) \
	X(None,              "None",               " ",     None)        \
	X(Percent,           "Percent",            "%",     None)        \
	X(PartsPerMillion,   "Parts Per Million",  "ppm",   None)        \
	X(Millivolt,         "Millivolt",          "mV",    Energy)      \
	X(Volt,              "Volt",               "V",     Energy)      \
	X(Milliampere,       "Milliampere",        "mA",    Energy)      \
	X(Ampere,            "Ampere",             "A",     Energy)      \
	X(Ohm,               "Ohm",                "Ω",     Energy)      \
	X(Milliohm,          "Milliohm",           "mΩ",    Energy)      \
	X(Kiloohm,           "Kiloohm",            "kΩ",    Energy)      \
	X(DielectricConstant,"Dielectric Constant"," ",     Energy)      \
	X(LitersPerSecond,   "Liters Per Second",  "L/s",   Flow)        \
	X(GallonsPerSecond,  "Gallons Per Second", "gal/s", Flow)        \
	X(Kilopascal,        "Kilopascal",         "kPa",   Pressure)    \
	X(Bar,               "Bar",                "bar",   Pressure)    \
	X(Pascal,            "Pascal",             "Pa",    Pressure)    \
	X(Torr,              "Torr",               "torr",  Pressure)    \
	X(Celsius,           "Celsius",            "°C",    Temperature) \
	X(Fahrenheit,        "Fahrenheit",         "°F",    Temperature) \
	X(Kelvin,            "Kelvin",             "K",     Temperature) \
	X(Lux,               "Lux",                "lx",    Light)       \
	X(Lumen,             "Lumen",              "lm",    Light)       \
	X(Millimeter,        "Millimeter",         "mm",    Length)      \
	X(Centimeter,        "Centimeter",         "cm",    Length)      \
	X(Meter,             "Meter",              "m",     Length)      \
	X(Kilometer,         "Kilometer",          "km",    Length)      \
	X(Inch,              "Inch",               "in",    Length)      \
	X(Foot,              "Foot",               "ft",    Length)      \
	X(Mile,              "Mile",               "mi",    Length)      \
	X(MetersPerSecond,   "Meters Per Second",  "m/s",   Velocity)    \
	X(KilometersPerHour, "Kilometers Per Hour","km/h",  Velocity)    \
	X(MilesPerHour,      "Miles Per Hour",     "mph",   Velocity)    \
	X(Milliliter,        "Milliliter",         "mL",    Volume)      \
	X(Liter,             "Liter",              "L",     Volume)      \
	X(CubicMeter,        "Cubic Meter",        "m³",    Volume)      \
	X(Gallon,            "Gallon",             "gal",   Volume)      \
	X(CubicFoot,         "Cubic Foot",         "ft³",   Volume)      \
	X(LitersPerMinute,   "Liters Per Minute",  "L/min", Flow)        \
	X(GallonsPerMinute,  "Gallons Per Minute", "gpm",   Flow)        \
	X(PoundsPerSquareInch, "Pounds Per Square Inch", "psi", Pressure) \
	X(VolumetricMoistureContent, "Volumetric Moisture Content", "%VMC", None) \
	X(Watt,              "Watt",               "W",     Energy)      \
	X(Milliwatt,         "Milliwatt",          "mW",    Energy)      \
	X(WattHour,          "Watt Hour",          "Wh",    Energy)      \
	X(KilowattHour,      "Kilowatt Hour",      "kWh",   Energy)      \
	X(MicrosiemensPerCentimeter, "Microsiemens Per Centimeter", "uS/cm", Salinity) \
	X(MillisiemensPerCentimeter, "Millisiemens Per Centimeter", "mS/cm", Salinity) \
	X(Ph,                "pH",                 "pH",    Salinity)    \
	X(Degree,            "Degree",             "deg",   Angle)       \
	X(Radian,            "Radian",             "rad",   Angle)       \
	X(MillimetersPerHour, "Millimeters Per Hour", "mm/h", Precipitation) \
	X(InchesPerHour,     "Inches Per Hour",    "in/h",  Precipitation) \
	X(Kilobyte,          "Kilobyte",           "KB",    None)

enum class CompatUnitGroup : uint8_t {
#define X(id, name) id,
	COMPAT_UNIT_GROUP_LIST(X)
#undef X
	MAX_VALUE,
};

enum class CompatUnit : uint8_t {
#define X(id, name, sym, group) id,
	COMPAT_UNIT_LIST(X)
#undef X
	MAX_VALUE,
};

// /jsn.status bits (runtime, never persisted)
#define COMPAT_STATUS_VALID        (1 << 0)
#define COMPAT_STATUS_ERROR        (1 << 1)
#define COMPAT_STATUS_STALE        (1 << 2)
#define COMPAT_STATUS_CLAMPED_HIGH (1 << 3)
#define COMPAT_STATUS_CLAMPED_LOW  (1 << 4)

// /jsn.flag and /csn.flag bits
#define COMPAT_FLAG_ENABLE (1 << 0)
#define COMPAT_FLAG_LOG    (1 << 1)
#define COMPAT_FLAG_SHOW   (1 << 2)

#define COMPAT_POINTS            8   // max piecewise points (sensor + program adjustment)
#define COMPAT_CHILDREN          8   // max aggregate children
#define COMPAT_NAME_LEN          33
#define COMPAT_UUID_NONE         0

#define COMPAT_DEFAULT_NAME      "New Sensor"
#define COMPAT_DEFAULT_INTERVAL  15      // minutes
#define COMPAT_DEFAULT_UNIT      CompatUnit::Volt
#define COMPAT_DEFAULT_MIN       0
#define COMPAT_DEFAULT_MAX       5
#define COMPAT_DEFAULT_TYPE      1       // ADS1115
#define COMPAT_DEFAULT_FLAG      COMPAT_FLAG_ENABLE

// Result codes of the mutation helpers; the HTTP layer maps them to HTML_*.
enum CompatResult : uint8_t {
	COMPAT_OK = 0,
	COMPAT_ERR_MISSING,
	COMPAT_ERR_OUTOFBOUND,
	COMPAT_ERR_FORMAT,
	COMPAT_ERR_NOSPACE,
	COMPAT_ERR_INTERNAL,
};

typedef SensorPoint_t CompatPoint;

// Parsed /csn request (has_* tells whether the parameter was present).
struct CompatCsnParams {
	bool is_new;
	uint nr;                 // target sensor nr (== uuid) when !is_new
	CompatSensorType type;

	bool has_name;     char name[COMPAT_NAME_LEN];
	bool has_min;      double min;
	bool has_max;      double max;
	bool has_interval; uint32_t interval;   // minutes
	bool has_unit;     CompatUnit unit;
	bool has_flag;     uint8_t flag;

	// Aggregate
	bool has_children; uint8_t nchildren; uint16_t child_uuid[COMPAT_CHILDREN];
	                   float child_scale[COMPAT_CHILDREN]; float child_offset[COMPAT_CHILDREN];
	bool has_action;   uint8_t action;      // AggregateAction or WeatherAction

	// ADS1115
	bool has_pin;      uint8_t pin;         // 1..16
	bool has_scale;    double scale;
	bool has_offset;   double offset;
	bool has_subtype;  uint8_t subtype;
	bool has_points;   uint8_t npoints; CompatPoint points[COMPAT_POINTS];

	// SystemInternal / OnboardDigital
	bool has_metric;   uint8_t metric;
	bool has_input;    uint8_t input;

	// Native (type 5)
	bool has_ntype;    uint ntype;
};

// --- enum/unit helpers ------------------------------------------------------
const char *compat_unit_name(CompatUnit unit);      // PROGMEM string
const char *compat_unit_short(CompatUnit unit);     // PROGMEM string
CompatUnitGroup compat_unit_group(CompatUnit unit);
const char *compat_enum_string(CompatUnitGroup g);
const char *compat_enum_string(CompatAggregateAction a);
const char *compat_enum_string(CompatWeatherAction a);

// --- our sensor -> upstream view --------------------------------------------
CompatSensorType compat_type_of(const SensorBase *s);
CompatUnit compat_unit_of(const SensorBase *s);
uint8_t compat_flag_of(const SensorBase *s);
uint8_t compat_status_of(const SensorBase *s, ulong now);
uint32_t compat_interval_of(const SensorBase *s);   // minutes, >= 1
void compat_range_of(const SensorBase *s, double &min, double &max);
bool compat_pin_of(const SensorBase *s, uint8_t &pin);   // 1-based ADS1115 pin

// Positional index (sid) <-> sensor nr (uuid)
SensorBase *compat_sensor_by_sid(uint sid);
int compat_sid_of(uint nr);   // -1 when not found

// --- JSON emitters (write into the global bfill) ----------------------------
// Emits one /jsn sensor object {"uuid":..,"extra":{..}} for s.
void compat_emit_sensor_json(BufferFiller &bfill, SensorBase *s, ulong now);
// Emits the /jsd body (everything after the opening '{', including the
// closing '}'). flush(ctx) is called between large chunks so the HTTP layer
// can push out the ether buffer.
typedef void (*CompatFlushFn)(void *ctx);
void compat_emit_sensor_desc_json(BufferFiller &bfill, CompatFlushFn flush, void *ctx);
// Emits the program "sensorAdjustment" object for 0-based pid ({} when none).
void compat_emit_program_adjust_json(BufferFiller &bfill, uint8_t pid);

// --- mutations --------------------------------------------------------------
// Add or modify a sensor from a parsed /csn request. On success out_nr holds
// the sensor nr (== uuid). Persists via the debounced sensor save.
CompatResult compat_change_sensor(CompatCsnParams &p, uint *out_nr);
// Delete one sensor (nr) or all sensors (nr == 0).
CompatResult compat_delete_sensor(uint nr);
// Apply a parsed "snadj" (flag, uuid, points) to 0-based program pid.
// flag bit0 == 0 && uuid == 0 && n == 0 clears every adjustment of the program.
CompatResult compat_snadj_apply(uint8_t pid, uint8_t flag, uint16_t uuid, const CompatPoint *points, uint8_t n);

// --- math -------------------------------------------------------------------
// Linear interpolation across (x, y) points, clamped at the endpoints.
// Duplicate x values form a step (x == T returns the rightmost point at T).
float compat_piecewise_interp(float x, const CompatPoint *points, uint8_t n);
// Program adjustment factor for PROG_PIECEWISE entries.
double compat_prog_adjust_piecewise(const ProgSensorAdjust *p, double sensorData);

#endif // _SENSOR_COMPAT_H
