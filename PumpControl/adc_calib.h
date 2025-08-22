/*
 * adc_calib.h
 *
 * Created: 22.08.2025 11:28:31
 *  Author: haasr
 */ 

#pragma once

// Für Ganzzahlvariante: feste Skalierung
static constexpr int32_t Scale = 100;   // z.B. feste Nachkommastellen
static constexpr int32_t Half  = Scale / 2;
	
/// Base Template (not implemented, just the frame)
/// phys = offset + slope * adc
template<typename T>
class AdcCalib;

/// Specialisation for float
template<>
class AdcCalib<float> {
	public:
	AdcCalib(uint16_t adc_max, float slope, float offset)
	: adc_max_(adc_max), slope_(slope), offset_(offset) {}

	float adcToPhys(uint16_t adc) const {
		return offset_ + slope_ * adc;
	}

	uint16_t physToAdc(float phys) const {
		float adc_f = (phys - offset_) / slope_;
		if (adc_f < 0) adc_f = 0;
		if (adc_f > adc_max_) adc_f = (float)adc_max_;
		return static_cast<uint16_t>(lround(adc_f));
		// return (uint16_t)lroundf(adc_f);
	}

	private:
	uint16_t adc_max_;
	float slope_;
	float offset_;
};

/// Specialisation for uint16_t
template<>
class AdcCalib<uint16_t> {
	public:
	AdcCalib(uint16_t adc_max, int16_t slope, int16_t offset)
	: adc_max_(adc_max), slope_(slope), offset_(offset) {}

	int16_t adcToPhys(uint16_t adc) const {
		// Round with +0.5
		return (offset_ + (slope_ * adc + Half) / Scale);
	}

	uint16_t physToAdc(int16_t phys) const {
		int32_t num = (static_cast<int32_t>(phys) - offset_) * Scale;
		if (num < 0) return 0;
		uint32_t adc = static_cast<uint32_t>(num) / slope_;
		if (adc > adc_max_) adc = adc_max_;
		return static_cast<uint16_t>(adc);
	}

	void setSlope(uint16_t slope){
		slope_ = slope;
	}
	
	void setOffset(const uint16_t offset){
		offset_ = offset;
	}
	
	private:
	uint16_t adc_max_;
	int16_t slope_;
	int16_t offset_;
};
