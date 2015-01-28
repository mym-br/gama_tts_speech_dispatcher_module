/***************************************************************************
 *  Copyright 2015 Marcelo Y. Matuda                                       *
 *                                                                         *
 *  This program is free software: you can redistribute it and/or modify   *
 *  it under the terms of the GNU General Public License as published by   *
 *  the Free Software Foundation, either version 3 of the License, or      *
 *  (at your option) any later version.                                    *
 *                                                                         *
 *  This program is distributed in the hope that it will be useful,        *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
 *  GNU General Public License for more details.                           *
 *                                                                         *
 *  You should have received a copy of the GNU General Public License      *
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.  *
 ***************************************************************************/

#include "ModuleConfiguration.h"

#include "Util.h"



ModuleConfiguration::ModuleConfiguration()
		: logLevel(0)
		, pitch(0.0)
		, rate(0.0)
		, volume(0.0)
		, punctuationMode(PUNCTUATION_NONE)
		, spellingMode(false)
		, capitalLetterRecognition(CAPITAL_LETTER_RECOGNITION_NONE)
{
}

void
ModuleConfiguration::setLogLevel(const std::string& s)
{
	unsigned int value = Util::convertString<unsigned int>(s);
	if (value > 5) value = 5;
	logLevel = value;
}

void
ModuleConfiguration::setPitch(const std::string& s)
{
	float value = Util::convertString<float>(s);
	if (value >  100.0f) value =  100.0f;
	if (value < -100.0f) value = -100.0f;
	pitch = value;
}

void
ModuleConfiguration::setRate(const std::string& s)
{
	float value = Util::convertString<float>(s);
	if (value >  100.0f) value =  100.0f;
	if (value < -100.0f) value = -100.0f;
	rate = value;
}

void
ModuleConfiguration::setVolume(const std::string& s)
{
	float value = Util::convertString<float>(s);
	if (value >  100.0f) value =  100.0f;
	if (value < -100.0f) value = -100.0f;
	volume = value;
}

void
ModuleConfiguration::setPunctuationMode(const std::string& s)
{
	if (s == "some") {
		punctuationMode = PUNCTUATION_SOME;
	} else if (s == "all") {
		punctuationMode = PUNCTUATION_ALL;
	} else { // "none"
		punctuationMode = PUNCTUATION_NONE;
	}
}

void
ModuleConfiguration::setSpellingMode(const std::string& s)
{
	if (s == "on") {
		spellingMode = true;
	} else { // "off"
		spellingMode = false;
	}
}

void
ModuleConfiguration::setCapitalLetterRecognition(const std::string& s)
{
	if (s == "spell") {
		capitalLetterRecognition = CAPITAL_LETTER_RECOGNITION_SPELL;
	} else if (s == "icon") {
		capitalLetterRecognition = CAPITAL_LETTER_RECOGNITION_ICON;
	} else { // "none"
		capitalLetterRecognition = CAPITAL_LETTER_RECOGNITION_NONE;
	}
}

void
ModuleConfiguration::setVoice(const std::string& s)
{
	if (s == "NULL") {
		voice = std::string();
	} else {
		voice = s;
	}
}

void
ModuleConfiguration::setLanguage(const std::string& s)
{
	if (s == "NULL") {
		language = std::string();
	} else {
		language = s;
	}
}

void
ModuleConfiguration::setSynthesisVoice(const std::string& s)
{
	if (s == "NULL") {
		synthesisVoice = std::string();
	} else {
		synthesisVoice = s;
	}
}
