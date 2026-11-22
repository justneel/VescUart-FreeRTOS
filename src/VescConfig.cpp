#include "VescConfig.h"

#include <math.h>
#include <string.h>

#include "buffer.h"

static uint32_t readSignature(const uint8_t * blob) {
	int32_t index = 0;
	return buffer_get_uint32(blob, &index);
}

size_t VescConfig::fieldSize(VescConfType type) {
	switch (type) {
		case VescConfType::U8: return 1;
		case VescConfType::U16:
		case VescConfType::I16:
		case VescConfType::F16: return 2;
		default: return 4;
	}
}

const VescConfLayout * VescConfig::findLayout(VescConfKind kind, uint32_t signature) {
	for (size_t i = 0; i < kVescConfLayoutCount; i++) {
		if (kVescConfLayouts[i].kind == kind && kVescConfLayouts[i].signature == signature) {
			return &kVescConfLayouts[i];
		}
	}
	return nullptr;
}

bool VescConfig::load(VescConfKind kind, const uint8_t * blob, size_t len) {
	layoutPtr = nullptr;
	length = 0;
	confKind = kind;
	blobSignature = len >= 4 ? readSignature(blob) : 0;
	if (len < 4 || len > MAX_SIZE) {
		return false;
	}

	const VescConfLayout * layout = findLayout(kind, blobSignature);
	if (layout == nullptr) {
		return false;
	}

	// The length must match the layout exactly, or the table is wrong for this firmware.
	size_t expected = 4;
	for (uint16_t i = 0; i < layout->count; i++) {
		expected += fieldSize(layout->fields[i].type);
	}
	if (expected != len) {
		return false;
	}

	memcpy(bytes, blob, len);
	length = len;
	layoutPtr = layout;
	return true;
}

const VescConfField * VescConfig::find(const char * name, size_t & offset) const {
	if (layoutPtr == nullptr) {
		return nullptr;
	}
	offset = 4;
	for (uint16_t i = 0; i < layoutPtr->count; i++) {
		const VescConfField & field = layoutPtr->fields[i];
		if (strcmp(field.name, name) == 0) {
			return &field;
		}
		offset += fieldSize(field.type);
	}
	return nullptr;
}

bool VescConfig::has(const char * name) const {
	size_t offset;
	return find(name, offset) != nullptr;
}

double VescConfig::decode(const VescConfField & field, const uint8_t * at) {
	int32_t index = 0;
	switch (field.type) {
		case VescConfType::U8: return at[0];
		case VescConfType::U16: return buffer_get_uint16(at, &index);
		case VescConfType::I16: return buffer_get_int16(at, &index);
		case VescConfType::U32: return buffer_get_uint32(at, &index);
		case VescConfType::I32: return buffer_get_int32(at, &index);
		case VescConfType::F16: return buffer_get_int16(at, &index) / (double) field.scale;
		case VescConfType::F32: return buffer_get_int32(at, &index) / (double) field.scale;
		case VescConfType::F32_AUTO: return buffer_get_float32_auto(at, &index);
	}
	return 0;
}

bool VescConfig::get(const char * name, double & value) const {
	size_t offset;
	const VescConfField * field = find(name, offset);
	if (field == nullptr) {
		return false;
	}
	value = decode(*field, bytes + offset);
	return true;
}

bool VescConfig::set(const char * name, double value, bool * changed) {
	size_t offset;
	const VescConfField * field = find(name, offset);
	if (field == nullptr || isnan(value) || isinf(value)) {
		return false;
	}

	uint8_t encoded[4];
	int32_t index = 0;
	switch (field->type) {
		case VescConfType::U8: {
			// Signed byte fields take negative values as two's complement.
			long v = lround(value);
			if (v < -128 || v > 255) return false;
			encoded[index++] = (uint8_t) v;
		} break;
		case VescConfType::U16: {
			long v = lround(value);
			if (v < 0 || v > 0xFFFF) return false;
			buffer_append_uint16(encoded, (uint16_t) v, &index);
		} break;
		case VescConfType::I16: {
			long v = lround(value);
			if (v < INT16_MIN || v > INT16_MAX) return false;
			buffer_append_int16(encoded, (int16_t) v, &index);
		} break;
		case VescConfType::U32: {
			double v = round(value);
			if (v < 0 || v > 4294967295.0) return false;
			buffer_append_uint32(encoded, (uint32_t) v, &index);
		} break;
		case VescConfType::I32: {
			double v = round(value);
			if (v < INT32_MIN || v > INT32_MAX) return false;
			buffer_append_int32(encoded, (int32_t) v, &index);
		} break;
		case VescConfType::F16: {
			// The firmware truncates; rounding keeps values like 0.1 from becoming 0.0999.
			double v = round(value * field->scale);
			if (v < INT16_MIN || v > INT16_MAX) return false;
			buffer_append_int16(encoded, (int16_t) v, &index);
		} break;
		case VescConfType::F32: {
			double v = round(value * field->scale);
			if (v < INT32_MIN || v > INT32_MAX) return false;
			buffer_append_int32(encoded, (int32_t) v, &index);
		} break;
		case VescConfType::F32_AUTO:
			buffer_append_float32_auto(encoded, (float) value, &index);
			break;
	}

	const bool differs = memcmp(bytes + offset, encoded, index) != 0;
	memcpy(bytes + offset, encoded, index);
	if (changed != nullptr) {
		*changed = differs;
	}
	return true;
}
