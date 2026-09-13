/* OpenSprinkler Unified (AVR/RPI/BBB/LINUX) Firmware
 * Copyright (C) 2015 by Ray Wang (ray@opensprinkler.com)
 * Analog Sensor API by Stefan Schmaltz (info@opensprinklershop.de)
 *
 * Upstream "Expanded Sensor" API compatibility layer (see sensor_compat.h)
 * Sep 2026 @ OpenSprinklerShop
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

#include "sensor_compat.h"
#include "SensorBase.hpp"
#include "OpenSprinkler.h"
#include "opensprinkler_server.h"
#include "ArduinoJson.hpp"
#include <math.h>
#include <string.h>
#include <stdio.h>

extern OpenSprinkler os;
extern ProgramData pd;

// ---------------------------------------------------------------------------
// enum / unit tables
// ---------------------------------------------------------------------------

const char *compat_unit_name(CompatUnit unit) {
	switch (unit) {
	#define X(id, name, sym, group) case CompatUnit::id: return PSTR(name);
	COMPAT_UNIT_LIST(X)
	#undef X
	default: return PSTR("");
	}
}

const char *compat_unit_short(CompatUnit unit) {
	switch (unit) {
	#define X(id, name, sym, group) case CompatUnit::id: return PSTR(sym);
	COMPAT_UNIT_LIST(X)
	#undef X
	default: return PSTR("");
	}
}

CompatUnitGroup compat_unit_group(CompatUnit unit) {
	switch (unit) {
	#define X(id, name, sym, group) case CompatUnit::id: return CompatUnitGroup::group;
	COMPAT_UNIT_LIST(X)
	#undef X
	default: return CompatUnitGroup::None;
	}
}

const char *compat_enum_string(CompatUnitGroup g) {
	switch (g) {
	#define X(id, name) case CompatUnitGroup::id: return PSTR(name);
	COMPAT_UNIT_GROUP_LIST(X)
	#undef X
	default: return nullptr;
	}
}

const char *compat_enum_string(CompatAggregateAction a) {
	switch (a) {
	#define X(id, name) case CompatAggregateAction::id: return PSTR(name);
	COMPAT_AGGREGATE_ACTION_LIST(X)
	#undef X
	default: return nullptr;
	}
}

const char *compat_enum_string(CompatWeatherAction a) {
	switch (a) {
	#define X(id, name, ntype) case CompatWeatherAction::id: return PSTR(name);
	COMPAT_WEATHER_ACTION_LIST(X)
	#undef X
	default: return nullptr;
	}
}

static uint compat_weather_native_type(CompatWeatherAction a) {
	switch (a) {
	#define X(id, name, ntype) case CompatWeatherAction::id: return ntype;
	COMPAT_WEATHER_ACTION_LIST(X)
	#undef X
	default: return 0;
	}
}

static int compat_weather_action_of(uint ntype) {
	#define X(id, name, nt) if (ntype == (uint)(nt)) return (int)CompatWeatherAction::id;
	COMPAT_WEATHER_ACTION_LIST(X)
	#undef X
	return -1;
}

// ---------------------------------------------------------------------------
// small emit helpers
// ---------------------------------------------------------------------------

// Compact number formatting ("%g" style) — BufferFiller's $E pads to 10.6.
static void emit_num(BufferFiller &bfill, double v) {
	char buf[24];
	if (!isfinite(v)) v = 0;
	snprintf(buf, sizeof(buf), "%g", v);
	bfill.emit_p(PSTR("$S"), buf);
}

static void emit_json_escaped(BufferFiller &bfill, const char *s) {
	if (!s) return;
	char out[2 * COMPAT_NAME_LEN + 8];
	size_t o = 0;
	for (const char *p = s; *p && o < sizeof(out) - 7; p++) {
		unsigned char c = (unsigned char)*p;
		switch (c) {
			case '"':  out[o++] = '\\'; out[o++] = '"'; break;
			case '\\': out[o++] = '\\'; out[o++] = '\\'; break;
			case '\n': out[o++] = '\\'; out[o++] = 'n'; break;
			case '\r': out[o++] = '\\'; out[o++] = 'r'; break;
			case '\t': out[o++] = '\\'; out[o++] = 't'; break;
			default:
				if (c < 0x20) { o += snprintf(out + o, sizeof(out) - o, "\\u%04x", c); }
				else out[o++] = (char)c;
		}
	}
	out[o] = 0;
	bfill.emit_p(PSTR("$S"), out);
}

// ---------------------------------------------------------------------------
// type mapping helpers
// ---------------------------------------------------------------------------

static bool compat_is_ads_type(uint type) {
	switch (type) {
		case SENSOR_ANALOG_EXTENSION_BOARD:
		case SENSOR_ANALOG_EXTENSION_BOARD_P:
		case SENSOR_ANALOG_PIECEWISE:
		case SENSOR_SMT50_MOIS:
		case SENSOR_SMT50_TEMP:
		case SENSOR_SMT100_ANALOG_MOIS:
		case SENSOR_SMT100_ANALOG_TEMP:
		case SENSOR_VH400:
		case SENSOR_THERM200:
		case SENSOR_AQUAPLUMB:
		case SENSOR_USERDEF:
		case SENSOR_OSPI_ANALOG:
		case SENSOR_OSPI_ANALOG_P:
		case SENSOR_OSPI_ANALOG_SMT50_MOIS:
		case SENSOR_OSPI_ANALOG_SMT50_TEMP:
			return true;
	}
	return false;
}

static bool compat_is_ospi_ads_type(uint type) {
	return type >= OSPI_SENSORS_START && type <= OSPI_SENSORS_END && type != SENSOR_INTERNAL_TEMP &&
	       type != SENSOR_ONBOARD_DIGITAL;
}

// Subtype our analog type maps to; scale/offset the LINEAR mapping implies.
static CompatAds1115Subtype compat_subtype_of(const SensorBase *s, double &scale, double &offset) {
	scale = 1; offset = 0;
	switch (s->type) {
		case SENSOR_ANALOG_EXTENSION_BOARD:
		case SENSOR_OSPI_ANALOG:
			if (s->lin_set) { scale = s->lin_scale; offset = s->lin_offset; }
			return CompatAds1115Subtype::LINEAR;
		case SENSOR_ANALOG_EXTENSION_BOARD_P:
		case SENSOR_OSPI_ANALOG_P:
			scale = 100.0 / 3.3;
			return CompatAds1115Subtype::LINEAR;
		case SENSOR_USERDEF:
			if (s->lin_set) { scale = s->lin_scale; offset = s->lin_offset; }
			else {
				// legacy: v' = (v - offset_mv/1000) * factor/divider + offset2/100
				double f = 1;
				if (s->factor && s->divider) f = (double)s->factor / (double)s->divider;
				else if (s->divider) f = 1.0 / (double)s->divider;
				else if (s->factor) f = (double)s->factor;
				scale = f;
				offset = (double)s->offset2 / 100.0 - f * (double)s->offset_mv / 1000.0;
			}
			return CompatAds1115Subtype::LINEAR;
		case SENSOR_ANALOG_PIECEWISE:      return CompatAds1115Subtype::PIECEWISE_LINEAR;
		case SENSOR_SMT50_MOIS:
		case SENSOR_OSPI_ANALOG_SMT50_MOIS: return CompatAds1115Subtype::SMT50_MOISTURE;
		case SENSOR_SMT50_TEMP:
		case SENSOR_OSPI_ANALOG_SMT50_TEMP: return CompatAds1115Subtype::SMT50_TEMPERATURE;
		case SENSOR_SMT100_ANALOG_MOIS:    return CompatAds1115Subtype::SMT100_MOISTURE;
		case SENSOR_SMT100_ANALOG_TEMP:    return CompatAds1115Subtype::SMT100_TEMPERATURE;
		case SENSOR_VH400:                 return CompatAds1115Subtype::VH400_MOISTURE;
		case SENSOR_THERM200:              return CompatAds1115Subtype::THERM200_TEMPERATURE;
		case SENSOR_AQUAPLUMB:             return CompatAds1115Subtype::AQUAPLUMB_LEVEL;
	}
	return CompatAds1115Subtype::LINEAR;
}

// Baked subtypes apply the universal scale/offset trim as well (upstream:
// y = scale * transform(v) + offset); ours use lin_scale/lin_offset for that.
static void compat_trim_of(const SensorBase *s, CompatAds1115Subtype st, double &scale, double &offset) {
	if (st == CompatAds1115Subtype::LINEAR) return;   // already resolved
	if (st == CompatAds1115Subtype::PIECEWISE_LINEAR) { scale = 1; offset = 0; return; }
	if (s->lin_set) { scale = s->lin_scale; offset = s->lin_offset; }
	else { scale = 1; offset = 0; }
}

CompatSensorType compat_type_of(const SensorBase *s) {
	if (!s) return CompatSensorType::Native;
	if (sensor_isgroup(s)) return CompatSensorType::Aggregate;
	if (compat_is_ads_type(s->type)) return CompatSensorType::ADS1115;
	if (compat_weather_action_of(s->type) >= 0) return CompatSensorType::Weather;
	switch (s->type) {
		case SENSOR_FREE_MEMORY:
		case SENSOR_FREE_STORE:
		case SENSOR_INTERNAL_TEMP:
			return CompatSensorType::SystemInternal;
		case SENSOR_ONBOARD_DIGITAL:
			return CompatSensorType::OnboardDigital;
	}
	return CompatSensorType::Native;
}

// Native unit of a baked ADS1115 subtype (None = user-defined)
static CompatUnit compat_subtype_native_unit(CompatAds1115Subtype st) {
	switch (st) {
		case CompatAds1115Subtype::SMT50_TEMPERATURE:
		case CompatAds1115Subtype::SMT100_TEMPERATURE:
		case CompatAds1115Subtype::THERM200_TEMPERATURE:
			return CompatUnit::Celsius;
		case CompatAds1115Subtype::SMT50_MOISTURE:
		case CompatAds1115Subtype::SMT100_MOISTURE:
		case CompatAds1115Subtype::VH400_MOISTURE:
			return CompatUnit::VolumetricMoistureContent;
		case CompatAds1115Subtype::SMT100_PERMITTIVITY:
			return CompatUnit::DielectricConstant;
		case CompatAds1115Subtype::AQUAPLUMB_LEVEL:
			return CompatUnit::Percent;
		default:
			return CompatUnit::None;
	}
}

// Find a CompatUnit whose short symbol equals sym (used for UNIT_USERDEF).
static bool compat_unit_from_symbol(const char *sym, CompatUnit &out) {
	if (!sym || !sym[0]) return false;
	for (uint8_t i = 1; i < (uint8_t)CompatUnit::MAX_VALUE; i++) {
		if (strcmp_P(sym, compat_unit_short((CompatUnit)i)) == 0) { out = (CompatUnit)i; return true; }
	}
	return false;
}

CompatUnit compat_unit_of(const SensorBase *s) {
	if (!s) return CompatUnit::None;
	unsigned char uid = s->getUnitId();
	switch (uid) {
		case UNIT_PERCENT:
			switch (s->type) {
				case SENSOR_AQUAPLUMB:
				case SENSOR_OSPI_ANALOG_P:
					return CompatUnit::Percent;
				default:
					return CompatUnit::VolumetricMoistureContent;
			}
		case UNIT_HUM_PERCENT:
		case UNIT_LEVEL:       return CompatUnit::Percent;
		case UNIT_DEGREE:      return CompatUnit::Celsius;
		case UNIT_FAHRENHEIT:  return CompatUnit::Fahrenheit;
		case UNIT_VOLT:        return CompatUnit::Volt;
		case UNIT_INCH:        return CompatUnit::Inch;
		case UNIT_MM:          return CompatUnit::Millimeter;
		case UNIT_MPH:         return CompatUnit::MilesPerHour;
		case UNIT_KMH:         return CompatUnit::KilometersPerHour;
		case UNIT_DK:          return CompatUnit::DielectricConstant;
		case UNIT_LM:          return CompatUnit::Lumen;
		case UNIT_LX:          return CompatUnit::Lux;
		case UNIT_LITER:
		case UNIT_LITER_CONSUMPTION:  return CompatUnit::Liter;
		case UNIT_GALLON:
		case UNIT_GALLON_CONSUMPTION: return CompatUnit::Gallon;
		case UNIT_USERDEF: {
			if (s->type == SENSOR_FREE_MEMORY || s->type == SENSOR_FREE_STORE) return CompatUnit::Kilobyte;
			CompatUnit u;
			if (compat_unit_from_symbol(s->getUserdefUnit(), u)) return u;
			return CompatUnit::None;
		}
	}
	return CompatUnit::None;
}

// Reverse mapping for user-defined units: our unitid, or UNIT_USERDEF with
// the short symbol as custom unit string.
static unsigned char compat_native_unitid(CompatUnit u) {
	switch (u) {
		case CompatUnit::None:       return UNIT_NONE;
		case CompatUnit::Percent:    return UNIT_LEVEL;
		case CompatUnit::VolumetricMoistureContent: return UNIT_PERCENT;
		case CompatUnit::Celsius:    return UNIT_DEGREE;
		case CompatUnit::Fahrenheit: return UNIT_FAHRENHEIT;
		case CompatUnit::Volt:       return UNIT_VOLT;
		case CompatUnit::Inch:       return UNIT_INCH;
		case CompatUnit::Millimeter: return UNIT_MM;
		case CompatUnit::MilesPerHour:      return UNIT_MPH;
		case CompatUnit::KilometersPerHour: return UNIT_KMH;
		case CompatUnit::DielectricConstant: return UNIT_DK;
		case CompatUnit::Lumen:      return UNIT_LM;
		case CompatUnit::Lux:        return UNIT_LX;
		case CompatUnit::Liter:      return UNIT_LITER;
		case CompatUnit::Gallon:     return UNIT_GALLON;
		default:                     return UNIT_USERDEF;
	}
}

uint8_t compat_flag_of(const SensorBase *s) {
	uint8_t f = 0;
	if (s->flags.enable) f |= COMPAT_FLAG_ENABLE;
	if (s->flags.log)    f |= COMPAT_FLAG_LOG;
	if (s->flags.show)   f |= COMPAT_FLAG_SHOW;
	return f;
}

uint8_t compat_status_of(const SensorBase *s, ulong now) {
	uint8_t st = 0;
	if (s->flags.data_ok) {
		st |= COMPAT_STATUS_VALID;
		if (s->flags.clamped_hi) st |= COMPAT_STATUS_CLAMPED_HIGH;
		if (s->flags.clamped_lo) st |= COMPAT_STATUS_CLAMPED_LOW;
		if (s->read_interval && s->last_read && now > s->last_read + 3UL * s->read_interval + 60UL)
			st |= COMPAT_STATUS_STALE;
	} else if (s->flags.enable && s->last_read) {
		st |= COMPAT_STATUS_ERROR;
	}
	return st;
}

uint32_t compat_interval_of(const SensorBase *s) {
	uint32_t m = (s->read_interval + 30) / 60;
	return m ? m : 1;
}

// Natural output range of a sensor when no explicit clamp is configured.
static void compat_default_range(const SensorBase *s, double &min, double &max) {
	min = 0; max = 100;
	switch (compat_type_of(s)) {
		case CompatSensorType::ADS1115: {
			double sc, of;
			CompatAds1115Subtype st = compat_subtype_of(s, sc, of);
			switch (st) {
				case CompatAds1115Subtype::LINEAR:
					if (s->type == SENSOR_ANALOG_EXTENSION_BOARD_P || s->type == SENSOR_OSPI_ANALOG_P) { min = 0; max = 100; }
					else if (s->type == SENSOR_USERDEF) { min = -1000; max = 1000; }
					else { min = 0; max = 5; }
					break;
				case CompatAds1115Subtype::SMT50_TEMPERATURE:
				case CompatAds1115Subtype::SMT100_TEMPERATURE:
				case CompatAds1115Subtype::THERM200_TEMPERATURE:
					min = -40; max = 185; break;
				case CompatAds1115Subtype::SMT50_MOISTURE:
					min = 0; max = 50; break;
				case CompatAds1115Subtype::SMT100_PERMITTIVITY:
					min = 1; max = 80; break;
				default:
					min = 0; max = 100; break;
			}
			break;
		}
		case CompatSensorType::SystemInternal:
			if (s->type == SENSOR_INTERNAL_TEMP) { min = -40; max = 185; }
			else if (s->type == SENSOR_FREE_MEMORY) { min = 0; max = 1000; }
			else { min = 0; max = 100000; }
			break;
		case CompatSensorType::OnboardDigital:
			min = 0; max = 1; break;
		case CompatSensorType::Weather:
			min = -1000; max = 1000; break;
		default:
			switch (s->getUnitId()) {
				case UNIT_DEGREE:
				case UNIT_FAHRENHEIT: min = -40; max = 185; break;
				case UNIT_PERCENT:
				case UNIT_HUM_PERCENT:
				case UNIT_LEVEL:      min = 0; max = 100; break;
				case UNIT_VOLT:       min = 0; max = 5; break;
				case UNIT_DK:         min = 1; max = 80; break;
				case UNIT_MM:
				case UNIT_INCH:
				case UNIT_MPH:
				case UNIT_KMH:
				case UNIT_LM:
				case UNIT_LX:         min = 0; max = 100000; break;
				default:              min = -1e6; max = 1e6; break;
			}
			break;
	}
}

void compat_range_of(const SensorBase *s, double &min, double &max) {
	if (s->clamp_en) { min = s->clamp_min; max = s->clamp_max; return; }
	compat_default_range(s, min, max);
}

bool compat_pin_of(const SensorBase *s, uint8_t &pin) {
	if (!compat_is_ads_type(s->type)) return false;
	if (compat_is_ospi_ads_type(s->type)) {
		pin = (uint8_t)(s->id + 1);
	} else {
		uint board = (s->port >= ASB_BOARD_ADDR1a && s->port <= ASB_BOARD_ADDR2b) ? (s->port - ASB_BOARD_ADDR1a) : 0;
		pin = (uint8_t)(board * 4 + (s->id & 3) + 1);
	}
	return true;
}

SensorBase *compat_sensor_by_sid(uint sid) {
	uint i = 0;
	for (auto it = sensors_iterate_begin(); ; ) {
		SensorBase *s = sensors_iterate_next(it);
		if (!s) break;
		if (i++ == sid) return s;
	}
	return nullptr;
}

int compat_sid_of(uint nr) {
	int i = 0;
	for (auto it = sensors_iterate_begin(); ; ) {
		SensorBase *s = sensors_iterate_next(it);
		if (!s) break;
		if (s->nr == nr) return i;
		i++;
	}
	return -1;
}

static uint compat_next_free_nr() {
	uint max_nr = 0;
	for (auto it = sensors_iterate_begin(); ; ) {
		SensorBase *s = sensors_iterate_next(it);
		if (!s) break;
		if (s->nr > max_nr) max_nr = s->nr;
	}
	return max_nr + 1;
}

// Members of an aggregate (group) sensor: our membership model is
// member->group == target, where target is the group's own "group" number
// (shared mode) or the group's nr (legacy mode).
static uint compat_group_target(const SensorBase *g) {
	return g->group ? g->group : g->nr;
}

// ---------------------------------------------------------------------------
// /jsn emitter
// ---------------------------------------------------------------------------

static void compat_emit_extra_json(BufferFiller &bfill, SensorBase *s) {
	switch (compat_type_of(s)) {
		case CompatSensorType::Aggregate: {
			uint8_t action = (uint8_t)(s->type - SENSOR_GROUP_MIN);
			bfill.emit_p(PSTR("{\"action\":$D,\"children\":["), action);
			uint target = compat_group_target(s);
			bool shared = s->group != 0;
			uint8_t n = 0;
			for (auto it = sensors_iterate_begin(); n < COMPAT_CHILDREN; ) {
				SensorBase *m = sensors_iterate_next(it);
				if (!m) break;
				if (m->nr == s->nr || m->group != target) continue;
				if (shared && sensor_isgroup(m)) continue;
				if (n) bfill.emit_p(PSTR(","));
				bfill.emit_p(PSTR("{\"uuid\":$D,\"scale\":1,\"offset\":0}"), m->nr);
				n++;
			}
			for (; n < COMPAT_CHILDREN; n++) {
				if (n) bfill.emit_p(PSTR(","));
				bfill.emit_p(PSTR("{\"uuid\":0,\"scale\":1,\"offset\":0}"));
			}
			bfill.emit_p(PSTR("]}"));
			break;
		}
		case CompatSensorType::ADS1115: {
			double scale, offset;
			CompatAds1115Subtype st = compat_subtype_of(s, scale, offset);
			compat_trim_of(s, st, scale, offset);
			uint8_t pin = 1;
			compat_pin_of(s, pin);
			bfill.emit_p(PSTR("{\"pin\":$D,\"scale\":"), pin);
			emit_num(bfill, scale);
			bfill.emit_p(PSTR(",\"offset\":"));
			emit_num(bfill, offset);
			bfill.emit_p(PSTR(",\"subtype\":$D"), (uint8_t)st);
			if (st == CompatAds1115Subtype::PIECEWISE_LINEAR) {
				bfill.emit_p(PSTR(",\"points\":["));
				for (uint8_t i = 0; i < s->pw_n && s->pw_points; i++) {
					if (i) bfill.emit_p(PSTR(","));
					bfill.emit_p(PSTR("{\"x\":"));
					emit_num(bfill, s->pw_points[i].x);
					bfill.emit_p(PSTR(",\"y\":"));
					emit_num(bfill, s->pw_points[i].y);
					bfill.emit_p(PSTR("}"));
				}
				bfill.emit_p(PSTR("]"));
			}
			bfill.emit_p(PSTR("}"));
			break;
		}
		case CompatSensorType::Weather:
			bfill.emit_p(PSTR("{\"action\":$D}"), compat_weather_action_of(s->type));
			break;
		case CompatSensorType::SystemInternal: {
			uint8_t metric = (s->type == SENSOR_FREE_MEMORY) ? (uint8_t)CompatSystemMetric::FREE_HEAP :
			                 (s->type == SENSOR_FREE_STORE)  ? (uint8_t)CompatSystemMetric::FREE_FLASH :
			                                                   (uint8_t)CompatSystemMetric::CPU_TEMPERATURE;
			bfill.emit_p(PSTR("{\"metric\":$D}"), metric);
			break;
		}
		case CompatSensorType::OnboardDigital:
			bfill.emit_p(PSTR("{\"input\":$D}"), s->id & 3);
			break;
		default:
			bfill.emit_p(PSTR("{\"ntype\":$D}"), s->type);
			break;
	}
}

void compat_emit_sensor_json(BufferFiller &bfill, SensorBase *s, ulong now) {
	double min, max;
	compat_range_of(s, min, max);
	bfill.emit_p(PSTR("{\"uuid\":$D,\"name\":\""), s->nr);
	emit_json_escaped(bfill, s->getName());
	bfill.emit_p(PSTR("\",\"unit\":$D,\"flag\":$D,\"status\":$D,\"interval\":$L,\"min\":"),
		(uint8_t)compat_unit_of(s), compat_flag_of(s), compat_status_of(s, now), (ulong)compat_interval_of(s));
	emit_num(bfill, min);
	bfill.emit_p(PSTR(",\"max\":"));
	emit_num(bfill, max);
	bfill.emit_p(PSTR(",\"value\":"));
	emit_num(bfill, s->last_data);
	bfill.emit_p(PSTR(",\"type\":$D,\"extra\":"), (uint8_t)compat_type_of(s));
	compat_emit_extra_json(bfill, s);
	bfill.emit_p(PSTR("}"));
}

// ---------------------------------------------------------------------------
// /jsd emitter
// ---------------------------------------------------------------------------

static bool compat_ads_hardware_detected() {
	uint16_t b = get_asb_detected_boards();
#if defined(ESP8266) || defined(ESP32)
	return (b & (ASB_BOARD1 | ASB_BOARD2)) != 0;
#else
	return (b & (OSPI_ADS1115 | OSPI_PCF8591)) != 0;
#endif
}

static void compat_emit_desc_aggregate(BufferFiller &bfill) {
	bfill.emit_p(PSTR(
		"{\"n\":\"Aggregate Sensor\","
		"\"as\":["
			"{\"n\":\"Children\",\"a\":\"children\",\"t\":\"array::8\",\"e\":["
				"{\"n\":\"Sensor UUID\",\"a\":\"uuid\",\"t\":\"sensor\",\"d\":\"0\",\"indicator\":true},"
				"{\"n\":\"Scale\",\"a\":\"scale\",\"t\":\"float\",\"d\":\"1\"},"
				"{\"n\":\"Offset\",\"a\":\"offset\",\"t\":\"float\",\"d\":\"0\"}"
			"]},"
			"{\"n\":\"Action\",\"a\":\"action\",\"t\":\"enum::AggregateAction\",\"d\":\"2\"}"
		"]}"));
}

static void compat_emit_desc_ads1115(BufferFiller &bfill) {
#if defined(ESP8266) || defined(ESP32)
	const int max_pin = 16;
#else
	const int max_pin = 8;
#endif
	bfill.emit_p(PSTR("{\"n\":\"ADS1115 Sensor\",\"hwd\":$D,\"as\":["), compat_ads_hardware_detected() ? 1 : 0);
	bfill.emit_p(PSTR(
			"{\"n\":\"Pin\",\"a\":\"pin\",\"t\":\"int::[1,$D]\",\"d\":\"1\",\"h\":\"Analog pin number (1-$D)\"},"
			"{\"n\":\"Subtype\",\"a\":\"subtype\",\"t\":\"enum\",\"d\":\"0\",\"o\":["), max_pin, max_pin);
	bfill.emit_p(PSTR("{\"id\":0,\"l\":\"Linear\",\"hd\":[\"points\"]}"));
#if defined(ESP8266) || defined(ESP32)
	bfill.emit_p(PSTR(",{\"id\":1,\"l\":\"Piecewise Linear\",\"hd\":[\"scale\",\"offset\"]}"));
#endif
	bfill.emit_p(PSTR(",{\"id\":10,\"l\":\"SMT50 Temperature\",\"hd\":[\"points\"],\"dfl\":{\"unit\":$D,\"min\":\"-40\",\"max\":\"185\"},\"ug\":$D}"),
		(uint8_t)CompatUnit::Celsius, (uint8_t)CompatUnitGroup::Temperature);
	bfill.emit_p(PSTR(",{\"id\":11,\"l\":\"SMT50 Moisture\",\"hd\":[\"points\"],\"dfl\":{\"unit\":$D,\"max\":\"50\"},\"lk\":[\"unit\"]}"),
		(uint8_t)CompatUnit::VolumetricMoistureContent);
#if defined(ESP8266) || defined(ESP32)
	bfill.emit_p(PSTR(",{\"id\":12,\"l\":\"SMT100 (0-3V) Temperature\",\"hd\":[\"points\"],\"dfl\":{\"unit\":$D,\"min\":\"-40\",\"max\":\"185\"},\"ug\":$D}"),
		(uint8_t)CompatUnit::Celsius, (uint8_t)CompatUnitGroup::Temperature);
	bfill.emit_p(PSTR(",{\"id\":13,\"l\":\"SMT100 (0-3V) Moisture\",\"hd\":[\"points\"],\"dfl\":{\"unit\":$D,\"max\":\"100\"},\"lk\":[\"unit\"]}"),
		(uint8_t)CompatUnit::VolumetricMoistureContent);
	bfill.emit_p(PSTR(",{\"id\":15,\"l\":\"VH400 Soil Moisture\",\"hd\":[\"points\"],\"dfl\":{\"unit\":$D,\"max\":\"100\"},\"lk\":[\"unit\"]}"),
		(uint8_t)CompatUnit::VolumetricMoistureContent);
	bfill.emit_p(PSTR(",{\"id\":16,\"l\":\"THERM200 Temperature\",\"hd\":[\"points\"],\"dfl\":{\"unit\":$D,\"min\":\"-40\",\"max\":\"185\"},\"ug\":$D}"),
		(uint8_t)CompatUnit::Celsius, (uint8_t)CompatUnitGroup::Temperature);
	bfill.emit_p(PSTR(",{\"id\":17,\"l\":\"AquaPlumb Water Level\",\"hd\":[\"points\"],\"dfl\":{\"unit\":$D,\"max\":\"100\"},\"lk\":[\"unit\"]}"),
		(uint8_t)CompatUnit::Percent);
#endif
	bfill.emit_p(PSTR(
			"]},"
			"{\"n\":\"Scale\",\"a\":\"scale\",\"t\":\"float\",\"d\":\"1\",\"h\":\"Sensor Value = Scale * Voltage + Offset\"},"
			"{\"n\":\"Offset\",\"a\":\"offset\",\"t\":\"float\",\"d\":\"0\",\"h\":\"Applied after Scale\"},"
			"{\"n\":\"Points\",\"a\":\"points\",\"t\":\"points::[0,8]\",\"h\":\"Up to 8 (voltage, value) pairs, sorted by voltage\"}"
		"]}"));
}

static void compat_emit_desc_weather(BufferFiller &bfill) {
	bfill.emit_p(PSTR(
		"{\"n\":\"Weather Sensor\","
		"\"as\":["
			"{\"n\":\"Weather Information\",\"a\":\"action\",\"t\":\"enum::WeatherAction\",\"d\":\"1\"}"
		"]}"));
}

static void compat_emit_desc_system(BufferFiller &bfill) {
	bfill.emit_p(PSTR("{\"n\":\"System Internal\",\"as\":[{\"n\":\"Metric\",\"a\":\"metric\",\"t\":\"enum\","));
#if defined(ESP8266) || defined(ESP32)
	bfill.emit_p(PSTR("\"d\":\"0\",\"o\":["));
	bfill.emit_p(PSTR("{\"id\":0,\"l\":\"Free Heap\",\"dfl\":{\"unit\":$D,\"max\":\"1000\"},\"lk\":[\"unit\"]}"), (uint8_t)CompatUnit::Kilobyte);
	bfill.emit_p(PSTR(",{\"id\":1,\"l\":\"Free Flash\",\"dfl\":{\"unit\":$D,\"max\":\"100000\"},\"lk\":[\"unit\"]}"), (uint8_t)CompatUnit::Kilobyte);
#else
	bfill.emit_p(PSTR("\"d\":\"3\",\"o\":["));
#endif
#if defined(ESP32) || defined(OSPI)
	#if defined(ESP32)
	bfill.emit_p(PSTR(","));
	#endif
	bfill.emit_p(PSTR("{\"id\":3,\"l\":\"CPU Temperature\",\"dfl\":{\"unit\":$D,\"min\":\"-40\",\"max\":\"185\"},\"ug\":$D}"),
		(uint8_t)CompatUnit::Celsius, (uint8_t)CompatUnitGroup::Temperature);
#endif
	bfill.emit_p(PSTR("]}]}"));
}

static void compat_emit_desc_onboard(BufferFiller &bfill) {
	bfill.emit_p(PSTR(
		"{\"n\":\"Onboard Digital\",\"as\":[{\"n\":\"Input\",\"a\":\"input\",\"t\":\"enum\",\"d\":\"0\",\"o\":["
			"{\"id\":0,\"l\":\"SN1\",\"dfl\":{\"unit\":0,\"max\":\"1\"},\"lk\":[\"unit\"]},"
			"{\"id\":1,\"l\":\"SN2\",\"dfl\":{\"unit\":0,\"max\":\"1\"},\"lk\":[\"unit\"]}"
		"]}]}"));
}

static void compat_emit_desc_native(BufferFiller &bfill) {
	bfill.emit_p(PSTR(
		"{\"n\":\"OpenSprinklerShop sensor\","
		"\"as\":["
			"{\"n\":\"Native type\",\"a\":\"ntype\",\"t\":\"int::[1,65535]\",\"d\":\"1\","
			 "\"h\":\"OpenSprinklerShop sensor type number (see /sf). Type-specific settings are edited on the Analog Sensors page.\"}"
		"]}"));
}

template <typename T>
static void compat_emit_enum(BufferFiller &bfill, const char *name) {
	bfill.emit_p(PSTR("\"$F\":["), name);
	bool first = true;
	for (uint8_t i = 0; i < (uint8_t)T::MAX_VALUE; i++) {
		const char *str = compat_enum_string((T)i);
		if (!str) continue;
		if (!first) bfill.emit_p(PSTR(","));
		bfill.emit_p(PSTR("\"$F\""), str);
		first = false;
	}
	bfill.emit_p(PSTR("]"));
}

void compat_emit_sensor_desc_json(BufferFiller &bfill, CompatFlushFn flush, void *ctx) {
	bfill.emit_p(PSTR("\"sensors\":["));
	// IMPORTANT: array index == CompatSensorType value; never skip entries.
	for (uint8_t i = 0; i < (uint8_t)CompatSensorType::MAX_VALUE; i++) {
		if (i) bfill.emit_p(PSTR(","));
		switch ((CompatSensorType)i) {
			case CompatSensorType::Aggregate:      compat_emit_desc_aggregate(bfill); break;
			case CompatSensorType::ADS1115:        compat_emit_desc_ads1115(bfill); break;
			case CompatSensorType::Weather:        compat_emit_desc_weather(bfill); break;
			case CompatSensorType::SystemInternal: compat_emit_desc_system(bfill); break;
			case CompatSensorType::OnboardDigital: compat_emit_desc_onboard(bfill); break;
			case CompatSensorType::Native:         compat_emit_desc_native(bfill); break;
			default: bfill.emit_p(PSTR("{\"n\":\"\",\"dis\":1}")); break;
		}
		if (flush) flush(ctx);
	}

	// units: [id, name, short, group]
	bfill.emit_p(PSTR("],\"units\":["));
	for (uint8_t i = 0; i < (uint8_t)CompatUnit::MAX_VALUE; i++) {
		if (i) bfill.emit_p(PSTR(","));
		CompatUnit u = (CompatUnit)i;
		bfill.emit_p(PSTR("[$D,\"$F\",\"$F\",$D]"), i, compat_unit_name(u), compat_unit_short(u), (uint8_t)compat_unit_group(u));
		if ((i & 7) == 7 && flush) flush(ctx);
	}

	bfill.emit_p(PSTR("],\"enums\":{"));
	compat_emit_enum<CompatUnitGroup>(bfill, PSTR("SensorUnitGroup"));
	bfill.emit_p(PSTR(","));
	compat_emit_enum<CompatAggregateAction>(bfill, PSTR("AggregateAction"));
	bfill.emit_p(PSTR(","));
	compat_emit_enum<CompatWeatherAction>(bfill, PSTR("WeatherAction"));
	bfill.emit_p(PSTR("}"));
	if (flush) flush(ctx);

	bfill.emit_p(PSTR(
		",\"as\":["
		"{\"n\":\"Name\",\"a\":\"name\",\"t\":\"string::[1,32]\",\"d\":\"" COMPAT_DEFAULT_NAME "\"},"
		"{\"n\":\"Interval\",\"a\":\"interval\",\"t\":\"int::[1,any]\",\"d\":\"15\",\"h\":\"Sensor's update interval (in minutes)\"},"));
	bfill.emit_p(PSTR("{\"n\":\"Unit\",\"a\":\"unit\",\"t\":\"unit\",\"d\":\"$D\"},"), (uint8_t)COMPAT_DEFAULT_UNIT);
	bfill.emit_p(PSTR(
		"{\"n\":\"Min. Value\",\"a\":\"min\",\"t\":\"float\",\"d\":\"0\"},"
		"{\"n\":\"Max. Value\",\"a\":\"max\",\"t\":\"float\",\"d\":\"5\"},"
		"{\"n\":\"Type\",\"a\":\"type\",\"t\":\"type\",\"d\":\"1\"}"
		"]"));

	bfill.emit_p(PSTR(",\"flags\":[{\"n\":\"Enabled\",\"d\":1},{\"n\":\"Logging\",\"d\":0},{\"n\":\"Show on Home\",\"d\":0}]"));
	bfill.emit_p(PSTR("}"));
}

// ---------------------------------------------------------------------------
// program adjustment ("snadj") mapping
// ---------------------------------------------------------------------------

float compat_piecewise_interp(float x, const CompatPoint *points, uint8_t n) {
	if (n == 0 || !points) return NAN;
	if (x < points[0].x) return points[0].y;
	if (x >= points[n - 1].x) return points[n - 1].y;
	uint8_t i = 0;
	while (i + 1 < n - 1 && x >= points[i + 1].x) i++;
	const CompatPoint &l = points[i];
	const CompatPoint &r = points[i + 1];
	if (r.x == l.x) return r.y;
	return (x - l.x) / (r.x - l.x) * (r.y - l.y) + l.y;
}

double compat_prog_adjust_piecewise(const ProgSensorAdjust *p, double sensorData) {
	if (!p || !p->pw_points || p->pw_n == 0) return 1.0;
	float v = compat_piecewise_interp((float)sensorData, p->pw_points, p->pw_n);
	if (isnan(v)) return 1.0;
	if (v < 0) v = 0;
	return v;
}

// First (lowest nr) adjustment of a 0-based program, or nullptr.
static ProgSensorAdjust *compat_prog_adjust_of(uint8_t pid) {
	for (auto it = prog_adjust_iterate_begin(); ; ) {
		ProgSensorAdjust *p = prog_adjust_iterate_next(it);
		if (!p) break;
		if (p->prog == (uint)pid + 1) return p;
	}
	return nullptr;
}

// Derive the equivalent points of a legacy (non piecewise) adjustment.
static uint8_t compat_prog_adjust_points(const ProgSensorAdjust *p, CompatPoint *out) {
	if (p->pw_points && p->pw_n) {
		uint8_t n = p->pw_n > COMPAT_POINTS ? COMPAT_POINTS : p->pw_n;
		for (uint8_t i = 0; i < n; i++) out[i] = p->pw_points[i];
		return n;
	}
	float mn = (float)p->min, mx = (float)p->max, f1 = (float)p->factor1, f2 = (float)p->factor2;
	switch (p->type) {
		case PROG_LINEAR:
		case PROG_NONE:
			if (mx <= mn) return 0;
			out[0] = {mn, f1}; out[1] = {mx, f2};
			return 2;
		case PROG_DIGITAL_MIN:
			out[0] = {mn, f1}; out[1] = {mn, f2};
			return 2;
		case PROG_DIGITAL_MAX:
			out[0] = {mx, f1}; out[1] = {mx, f2};
			return 2;
		case PROG_DIGITAL_MINMAX:
			out[0] = {mn, f1}; out[1] = {mn, f2}; out[2] = {mx, f2}; out[3] = {mx, f1};
			return 4;
	}
	return 0;
}

void compat_emit_program_adjust_json(BufferFiller &bfill, uint8_t pid) {
	ProgSensorAdjust *p = compat_prog_adjust_of(pid);
	if (!p || p->type == PROG_DELETE) {
		bfill.emit_p(PSTR("{}"));
		return;
	}
	CompatPoint pts[COMPAT_POINTS];
	uint8_t n = compat_prog_adjust_points(p, pts);
	uint8_t flag = (p->type == PROG_NONE) ? 0 : 1;
	bfill.emit_p(PSTR("{\"flag\":$D,\"uuid\":$D,\"splits\":["), flag, p->sensor);
	for (uint8_t i = 0; i < n; i++) {
		if (i) bfill.emit_p(PSTR(","));
		bfill.emit_p(PSTR("{\"x\":"));
		emit_num(bfill, pts[i].x);
		bfill.emit_p(PSTR(",\"y\":"));
		emit_num(bfill, pts[i].y);
		bfill.emit_p(PSTR("}"));
	}
	bfill.emit_p(PSTR("]}"));
}

CompatResult compat_snadj_apply(uint8_t pid, uint8_t flag, uint16_t uuid, const CompatPoint *points, uint8_t n) {
	bool enabled = (flag & 1) != 0;

	// "snadj=0,0": clear every adjustment of this program
	if (!enabled && uuid == COMPAT_UUID_NONE && n == 0) {
		bool changed = false;
		for (;;) {
			ProgSensorAdjust *p = compat_prog_adjust_of(pid);
			if (!p) break;
			prog_adjust_delete(p->nr, false);
			changed = true;
		}
		if (changed) prog_adjust_save();
		return COMPAT_OK;
	}

	if (uuid == COMPAT_UUID_NONE) return COMPAT_ERR_OUTOFBOUND;
	if (!sensor_by_nr(uuid)) return COMPAT_ERR_OUTOFBOUND;

	ProgSensorAdjust *existing = compat_prog_adjust_of(pid);
	uint nr;
	if (existing) {
		nr = existing->nr;
	} else {
		nr = 1;
		for (auto it = prog_adjust_iterate_begin(); ; ) {
			ProgSensorAdjust *p = prog_adjust_iterate_next(it);
			if (!p) break;
			if (p->nr >= nr) nr = p->nr + 1;
		}
	}

	// Determine our adjustment type
	uint type;
	double mn = 0, mx = 0, f1 = 1, f2 = 1;
	bool store_points = false;
	if (!enabled) {
		type = PROG_NONE;
		store_points = n > 0;
	} else if (n == 0) {
		type = PROG_NONE;
	} else if (n == 2 && points[0].x < points[1].x) {
		type = PROG_LINEAR; mn = points[0].x; mx = points[1].x; f1 = points[0].y; f2 = points[1].y;
	} else if (n == 2 && points[0].x == points[1].x) {
		type = PROG_DIGITAL_MIN; mn = points[0].x; mx = points[0].x; f1 = points[0].y; f2 = points[1].y;
	} else if (n == 4 && points[0].x == points[1].x && points[2].x == points[3].x &&
	           points[1].x < points[2].x && points[0].y == points[3].y && points[1].y == points[2].y) {
		type = PROG_DIGITAL_MINMAX; mn = points[0].x; mx = points[2].x; f1 = points[0].y; f2 = points[1].y;
	} else {
		type = PROG_PIECEWISE; store_points = true;
		mn = points[0].x; mx = points[n - 1].x; f1 = points[0].y; f2 = points[n - 1].y;
	}
	if (!enabled && existing) {
		// keep the legacy parameters of a disabled adjustment
		mn = existing->min; mx = existing->max; f1 = existing->factor1; f2 = existing->factor2;
	}

	ArduinoJson::JsonDocument doc;
	ArduinoJson::JsonObject obj = doc.to<ArduinoJson::JsonObject>();
	obj["nr"] = nr;
	obj["type"] = type;
	obj["sensor"] = (uint)uuid;
	obj["prog"] = (uint)pid + 1;
	obj["factor1"] = f1;
	obj["factor2"] = f2;
	obj["min"] = mn;
	obj["max"] = mx;
	if (existing) {
		obj["stale_timeout"] = existing->stale_timeout;
		obj["stale_policy"] = existing->stale_policy;
		obj["stale_fallback"] = existing->stale_fallback;
		obj["order"] = existing->order;
		obj["name"] = existing->getName();
	} else {
		char name[24];
		snprintf(name, sizeof(name), "Program %d", (int)pid + 1);
		obj["name"] = name;
	}
	if (store_points) {
		ArduinoJson::JsonArray arr = obj["points"].to<ArduinoJson::JsonArray>();
		for (uint8_t i = 0; i < n; i++) {
			ArduinoJson::JsonArray pt = arr.add<ArduinoJson::JsonArray>();
			pt.add(points[i].x);
			pt.add(points[i].y);
		}
	}
	int ret = prog_adjust_define(obj, true);
	if (ret == HTTP_RQT_NOT_ENOUGH_SPACE) return COMPAT_ERR_NOSPACE;
	if (ret != HTTP_RQT_SUCCESS) return COMPAT_ERR_INTERNAL;
	return COMPAT_OK;
}

// ---------------------------------------------------------------------------
// /csn, /dsn
// ---------------------------------------------------------------------------

static CompatResult compat_resolve_ads_type(CompatCsnParams &p, SensorBase *existing, uint &ntype) {
	CompatAds1115Subtype st = CompatAds1115Subtype::LINEAR;
	double cur_scale = 1, cur_offset = 0;
	if (existing && compat_is_ads_type(existing->type)) {
		st = compat_subtype_of(existing, cur_scale, cur_offset);
		compat_trim_of(existing, st, cur_scale, cur_offset);
	}
	if (p.has_subtype) {
		if (p.subtype >= (uint8_t)CompatAds1115Subtype::MAX_VALUE) return COMPAT_ERR_OUTOFBOUND;
		st = (CompatAds1115Subtype)p.subtype;
	}
	double scale = p.has_scale ? p.scale : cur_scale;
	double offset = p.has_offset ? p.offset : cur_offset;
	bool trimmed = (fabs(scale - 1.0) > 1e-9) || (fabs(offset) > 1e-9);

#if defined(ESP8266) || defined(ESP32)
	switch (st) {
		case CompatAds1115Subtype::LINEAR:
			if (!trimmed) ntype = SENSOR_ANALOG_EXTENSION_BOARD;
			else if (existing && existing->type == SENSOR_ANALOG_EXTENSION_BOARD_P &&
			         fabs(scale - 100.0 / 3.3) < 1e-3 && fabs(offset) < 1e-9 && !p.has_unit)
				ntype = SENSOR_ANALOG_EXTENSION_BOARD_P;
			else ntype = SENSOR_USERDEF;
			break;
		case CompatAds1115Subtype::PIECEWISE_LINEAR: ntype = SENSOR_ANALOG_PIECEWISE; break;
		case CompatAds1115Subtype::SMT50_TEMPERATURE:    ntype = SENSOR_SMT50_TEMP; break;
		case CompatAds1115Subtype::SMT50_MOISTURE:       ntype = SENSOR_SMT50_MOIS; break;
		case CompatAds1115Subtype::SMT100_TEMPERATURE:   ntype = SENSOR_SMT100_ANALOG_TEMP; break;
		case CompatAds1115Subtype::SMT100_MOISTURE:      ntype = SENSOR_SMT100_ANALOG_MOIS; break;
		case CompatAds1115Subtype::VH400_MOISTURE:       ntype = SENSOR_VH400; break;
		case CompatAds1115Subtype::THERM200_TEMPERATURE: ntype = SENSOR_THERM200; break;
		case CompatAds1115Subtype::AQUAPLUMB_LEVEL:      ntype = SENSOR_AQUAPLUMB; break;
		default: return COMPAT_ERR_OUTOFBOUND;   // 14 (permittivity) and reserved values
	}
#else
	switch (st) {
		case CompatAds1115Subtype::LINEAR:
			if (existing && existing->type == SENSOR_OSPI_ANALOG_P &&
			    fabs(scale - 100.0 / 3.3) < 1e-3 && fabs(offset) < 1e-9 && !p.has_unit)
				ntype = SENSOR_OSPI_ANALOG_P;
			else ntype = SENSOR_OSPI_ANALOG;
			break;
		case CompatAds1115Subtype::SMT50_TEMPERATURE: ntype = SENSOR_OSPI_ANALOG_SMT50_TEMP; break;
		case CompatAds1115Subtype::SMT50_MOISTURE:    ntype = SENSOR_OSPI_ANALOG_SMT50_MOIS; break;
		default: return COMPAT_ERR_OUTOFBOUND;
	}
#endif
	if (!sensor_type_supported(ntype)) return COMPAT_ERR_OUTOFBOUND;

	// carry resolved values back so the caller can store them
	p.subtype = (uint8_t)st; p.has_subtype = true;
	p.scale = scale; p.offset = offset;
	p.has_scale = trimmed && !(ntype == SENSOR_ANALOG_EXTENSION_BOARD_P || ntype == SENSOR_OSPI_ANALOG_P);
	p.has_offset = p.has_scale;
	return COMPAT_OK;
}

CompatResult compat_change_sensor(CompatCsnParams &p, uint *out_nr) {
	SensorBase *existing = p.is_new ? nullptr : sensor_by_nr(p.nr);
	if (!p.is_new && !existing) return COMPAT_ERR_OUTOFBOUND;
	if (p.has_interval && p.interval < 1) return COMPAT_ERR_OUTOFBOUND;
	if (p.has_unit && p.unit >= CompatUnit::MAX_VALUE) return COMPAT_ERR_OUTOFBOUND;
	if (p.has_min && p.has_max && p.min > p.max) return COMPAT_ERR_OUTOFBOUND;

	uint nr = p.is_new ? compat_next_free_nr() : p.nr;
	uint ntype = 0;
	CompatUnit native_unit = CompatUnit::None;   // unit implied by the type (None = user-defined)

	switch (p.type) {
		case CompatSensorType::Aggregate: {
			uint8_t action = (uint8_t)CompatAggregateAction::Average;
			if (existing && sensor_isgroup(existing)) action = (uint8_t)(existing->type - SENSOR_GROUP_MIN);
			if (p.has_action) {
				if (p.action >= (uint8_t)CompatAggregateAction::MAX_VALUE) return COMPAT_ERR_OUTOFBOUND;
				action = p.action;
			}
			ntype = SENSOR_GROUP_MIN + action;
			if (p.has_children) {
				for (uint8_t i = 0; i < p.nchildren; i++) {
					if (p.child_uuid[i] == COMPAT_UUID_NONE) continue;
					if (fabs(p.child_scale[i] - 1.0f) > 1e-6f || fabs(p.child_offset[i]) > 1e-6f) return COMPAT_ERR_OUTOFBOUND;
					if (p.child_uuid[i] == nr) return COMPAT_ERR_OUTOFBOUND;
					if (!sensor_by_nr(p.child_uuid[i])) return COMPAT_ERR_OUTOFBOUND;
				}
			}
			break;
		}
		case CompatSensorType::ADS1115: {
			CompatResult r = compat_resolve_ads_type(p, existing, ntype);
			if (r != COMPAT_OK) return r;
			if (p.has_pin) {
#if defined(ESP8266) || defined(ESP32)
				if (p.pin < 1 || p.pin > 16) return COMPAT_ERR_OUTOFBOUND;
#else
				if (p.pin < 1 || p.pin > 8) return COMPAT_ERR_OUTOFBOUND;
#endif
			} else if (!existing || !compat_is_ads_type(existing->type)) {
				p.pin = 1; p.has_pin = true;
			}
			CompatAds1115Subtype st = (CompatAds1115Subtype)p.subtype;
			if (st == CompatAds1115Subtype::PIECEWISE_LINEAR) {
				uint8_t have = p.has_points ? p.npoints : ((existing && existing->type == SENSOR_ANALOG_PIECEWISE) ? existing->pw_n : 0);
				if (have < 2) return COMPAT_ERR_MISSING;
			}
			native_unit = compat_subtype_native_unit(st);
			break;
		}
		case CompatSensorType::Weather: {
			int action = -1;
			if (existing) action = compat_weather_action_of(existing->type);
			if (p.has_action) action = p.action;
			if (action < 0 || action >= (int)CompatWeatherAction::MAX_VALUE) return COMPAT_ERR_MISSING;
			ntype = compat_weather_native_type((CompatWeatherAction)action);
			break;
		}
		case CompatSensorType::SystemInternal: {
			int metric = -1;
			if (existing) {
				if (existing->type == SENSOR_FREE_MEMORY) metric = (int)CompatSystemMetric::FREE_HEAP;
				else if (existing->type == SENSOR_FREE_STORE) metric = (int)CompatSystemMetric::FREE_FLASH;
				else if (existing->type == SENSOR_INTERNAL_TEMP) metric = (int)CompatSystemMetric::CPU_TEMPERATURE;
			}
			if (p.has_metric) metric = p.metric;
			switch (metric) {
				case (int)CompatSystemMetric::FREE_HEAP:       ntype = SENSOR_FREE_MEMORY; break;
				case (int)CompatSystemMetric::FREE_FLASH:      ntype = SENSOR_FREE_STORE; break;
				case (int)CompatSystemMetric::CPU_TEMPERATURE: ntype = SENSOR_INTERNAL_TEMP; native_unit = CompatUnit::Celsius; break;
				case -1: return COMPAT_ERR_MISSING;
				default: return COMPAT_ERR_OUTOFBOUND;
			}
			if (!sensor_type_supported(ntype)) return COMPAT_ERR_OUTOFBOUND;
			break;
		}
		case CompatSensorType::OnboardDigital: {
			int input = -1;
			if (existing && existing->type == SENSOR_ONBOARD_DIGITAL) input = existing->id;
			if (p.has_input) input = p.input;
			if (input < 0) return COMPAT_ERR_MISSING;
			if (input > (int)CompatOnboardInput::SN2) return COMPAT_ERR_OUTOFBOUND;   // SN3/SN4: not available
			ntype = SENSOR_ONBOARD_DIGITAL;
			p.input = (uint8_t)input; p.has_input = true;
			break;
		}
		case CompatSensorType::Native: {
			if (p.has_ntype) ntype = p.ntype;
			else if (existing) ntype = existing->type;
			if (ntype == 0) return COMPAT_ERR_MISSING;
			if (!sensor_type_supported((int)ntype)) return COMPAT_ERR_OUTOFBOUND;
			break;
		}
		default:
			return COMPAT_ERR_OUTOFBOUND;
	}

	// Unit handling: baked types constrain the unit to their native unit.
	if (p.has_unit && native_unit != CompatUnit::None && p.unit != native_unit) {
		if (compat_unit_group(p.unit) != compat_unit_group(native_unit)) return COMPAT_ERR_OUTOFBOUND;
		// same group but a different unit (e.g. Fahrenheit): the firmware does
		// not convert units server-side; reject so the client keeps the native unit.
		return COMPAT_ERR_OUTOFBOUND;
	}

	bool partial = existing && existing->type == ntype;

	ArduinoJson::JsonDocument doc;
	ArduinoJson::JsonObject cfg = doc.to<ArduinoJson::JsonObject>();
	if (existing && !partial) {
		// The native type changes: sensor_define() recreates the object from
		// this JSON, so start from the complete persistent configuration of
		// the existing sensor (interval, address, flags, clamp, ...) and only
		// override what the request specifies.
		existing->toJson(cfg);
		cfg.remove("data_ok");
		cfg.remove("last");
		cfg.remove("nativedata");
		cfg.remove("data");
		cfg.remove("trend");
		if (existing->getUnitId() != UNIT_USERDEF) cfg.remove("unit");
	}
	cfg["nr"] = nr;
	if (!partial) cfg["type"] = ntype;

	if (p.has_name) cfg["name"] = p.name;
	else if (p.is_new) cfg["name"] = COMPAT_DEFAULT_NAME;

	if (p.has_interval) cfg["ri"] = p.interval * 60;
	else if (p.is_new) cfg["ri"] = COMPAT_DEFAULT_INTERVAL * 60;

	uint8_t flag = p.has_flag ? p.flag : (existing ? compat_flag_of(existing) : (uint8_t)COMPAT_DEFAULT_FLAG);
	cfg["enable"] = (flag & COMPAT_FLAG_ENABLE) ? 1 : 0;
	cfg["log"]    = (flag & COMPAT_FLAG_LOG) ? 1 : 0;
	cfg["show"]   = (flag & COMPAT_FLAG_SHOW) ? 1 : 0;

	// user-defined units (linear/piecewise analog, aggregate, native)
	if (p.has_unit && native_unit == CompatUnit::None &&
	    (p.type == CompatSensorType::ADS1115 || p.type == CompatSensorType::Aggregate ||
	     p.type == CompatSensorType::Native)) {
		unsigned char uid = compat_native_unitid(p.unit);
		cfg["unitid"] = uid;
		if (uid == UNIT_USERDEF) {
			char sym[16];
			strncpy_P(sym, compat_unit_short(p.unit), sizeof(sym) - 1);
			sym[sizeof(sym) - 1] = 0;
			cfg["unit"] = sym;
		}
	}

	// clamping range
	if (p.has_min || p.has_max) {
		double dmin = 0, dmax = 0;
		if (existing) compat_range_of(existing, dmin, dmax);
		else { dmin = COMPAT_DEFAULT_MIN; dmax = COMPAT_DEFAULT_MAX; }
		cfg["cmin"] = p.has_min ? p.min : dmin;
		cfg["cmax"] = p.has_max ? p.max : dmax;
		cfg["clamp"] = 1;
	}

	// type specific
	if (p.type == CompatSensorType::ADS1115) {
		if (p.has_pin) {
#if defined(ESP8266) || defined(ESP32)
			cfg["port"] = ASB_BOARD_ADDR1a + (p.pin - 1) / 4;
			cfg["id"] = (p.pin - 1) % 4;
#else
			cfg["id"] = p.pin - 1;
#endif
		}
		if (p.has_scale || p.has_offset) {
			cfg["lscale"] = p.scale;
			cfg["loffset"] = p.offset;
			cfg["lset"] = 1;
		} else if (!partial) {
			cfg["lset"] = 0;
		}
		if (p.has_points) {
			ArduinoJson::JsonArray arr = cfg["points"].to<ArduinoJson::JsonArray>();
			for (uint8_t i = 0; i < p.npoints; i++) {
				ArduinoJson::JsonArray pt = arr.add<ArduinoJson::JsonArray>();
				pt.add(p.points[i].x);
				pt.add(p.points[i].y);
			}
		}
	} else if (p.type == CompatSensorType::OnboardDigital) {
		cfg["id"] = p.input;
	} else if (p.type == CompatSensorType::Aggregate && !partial) {
		cfg["group"] = 0;   // legacy membership mode: members reference our nr
	}

	int ret = sensor_define(cfg, false);
	if (ret == HTTP_RQT_NOT_ENOUGH_SPACE) return COMPAT_ERR_NOSPACE;
	if (ret != HTTP_RQT_SUCCESS) return COMPAT_ERR_INTERNAL;

	// Aggregate membership
	if (p.type == CompatSensorType::Aggregate && p.has_children) {
		SensorBase *g = sensor_by_nr(nr);
		if (g) {
			uint target = compat_group_target(g);
			bool shared = g->group != 0;
			for (auto it = sensors_iterate_begin(); ; ) {
				SensorBase *m = sensors_iterate_next(it);
				if (!m) break;
				if (m->nr == nr) continue;
				bool wanted = false;
				for (uint8_t i = 0; i < p.nchildren; i++) if (p.child_uuid[i] == m->nr) wanted = true;
				if (wanted) m->group = target;
				else if (!shared && m->group == target) m->group = 0;
			}
		}
	}

	sensor_request_save();
	if (out_nr) *out_nr = nr;
	return COMPAT_OK;
}

CompatResult compat_delete_sensor(uint nr) {
	if (nr == 0) {
		uint nrs[64];
		for (;;) {
			uint n = 0;
			for (auto it = sensors_iterate_begin(); n < 64; ) {
				SensorBase *s = sensors_iterate_next(it);
				if (!s) break;
				nrs[n++] = s->nr;
			}
			if (n == 0) break;
			for (uint i = 0; i < n; i++) sensor_delete(nrs[i], false);
		}
		sensor_request_save();
		return COMPAT_OK;
	}
	if (!sensor_by_nr(nr)) return COMPAT_ERR_OUTOFBOUND;
	if (sensor_delete(nr, false) != HTTP_RQT_SUCCESS) return COMPAT_ERR_INTERNAL;
	sensor_request_save();
	return COMPAT_OK;
}
