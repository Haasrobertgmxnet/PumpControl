/*
 * adc_calib_maintainer.h
 *
 * Created: 22.08.2025 15:27:32
 *  Author: haasr
 */ 

#pragma once

#include <avr/eeprom.h>
#include "adc_calib.h"
	
#undef ENSURE_RVO // in case of C++17 this hopefully not needed, otherwise
// #define ENSURE_RVO

template<typename T>
class AdcCalibMaintainer {
	public:
	AdcCalibMaintainer(uint16_t adc_1, T val_1, uint16_t adc_2, T val_2)
	: adc_1_(adc_1<adc_2?adc_1:adc_2), 
	val_1_(adc_1<adc_2?val_1:val_2), 
	adc_2_(adc_1<adc_2?adc_2:adc_1), 
	val_2_(adc_1<adc_2?val_2:val_1) {
		if(adc_2_== adc_1_) return;
		slope_ =(val_2_ - val_1_)/(adc_2_-adc_1_);
		offset_ = val_1_ - slope_*adc_1_;
	}
	
	/// Write slope and offset to EEPROM
	void SaveToEEPROM(uint16_t addr) const {
		eeprom_update_block((const void*)&slope_, (void*)addr, sizeof(T));
		eeprom_update_block((const void*)&offset_, (void*)(addr + sizeof(T)), sizeof(T));
	}

	/// Load slope and offset from EEPROM
	void LoadFromEEPROM(uint16_t addr) {
		eeprom_read_block((void*)&slope_, (const void*)addr, sizeof(T));
		eeprom_read_block((void*)&offset_, (const void*)(addr + sizeof(T)), sizeof(T));
	}
	
#ifdef ENSURE_RVO
	// If you are not sure, if RVO is applied you may
	// define an ENSURE_RVO macro by #define ASSURE_RVO
	// to make sure that a initializer list is returned 
	// to construct the returned AdcCalib object
	// !!! This compiling path produces a warning -Wnarrowing
	AdcCalib<T> Create(uint16_t adc_max = 1023) {
		return { adc_max, slope_, offset_ };
	}
#else
	// If the ENSURE_RVO macro is not defined
	// In case of C++17 standard (std=c++17) 
	// RVO is mandatory, thus ENSURE_RVO is obsolete
	AdcCalib<T> Create(uint16_t adc_max= 1023){
		return AdcCalib<T>(adc_max, slope_, offset_);
	}
#endif
	
	void ApplyCalibration(AdcCalib<T>& adcCalib){
		adcCalib.setSlope(slope_);
		adcCalib.setOffset(offset_);
	}
	
	void ReCalculateStraightLine(){
		if(adc_2_== adc_1_) return;
		slope_ = (val_2_ - val_1_)/(adc_2_ - adc_1_);
		offset_ = val_2_ - slope_*adc_2_;
	}

	void ReCalibrateLower(uint16_t adc_1, T val_1){
		adc_1_ = adc_1;
		val_1_ = val_1;
		ReCalculateStraightLine();
	}

	void ReCalibrateUpper(uint16_t adc_2, T val_2){
		adc_2_ = adc_2;
		val_2_ = val_2;
		ReCalculateStraightLine();
	}
	
	private:
	// Point 1 == lower ADC value
	uint16_t adc_1_;
	T val_1_;
	// Point 2 == upper ADC value
	uint16_t adc_2_;
	T val_2_;
	T slope_;
	T offset_;
};
