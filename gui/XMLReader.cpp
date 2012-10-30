#include "XMLReader.h"

XMLReader::XMLReader(){
	_path = "";
	_size = 0;
}

XMLReader::XMLReader(std::string path){
	_path = path;
	_size = 0;
	tinyxml2::XMLDocument doc;
	if( doc.LoadFile(_path.c_str()) == 0 ){
		tinyxml2::XMLElement * iElem = doc.FirstChildElement();
		for(iElem;iElem;iElem=iElem->NextSiblingElement()){
			_size++;
			Data iData;
			iData.tag = iElem->Name();
			iData.value = iElem->GetText();
			_data.push_back(iData);
		}
	}
}

bool XMLReader::save(std::string path){
	if(path != ""){
		_path = path;
	}
	tinyxml2::XMLDocument doc;
	for(int iData = 0; iData < _data.size(); iData++){
		tinyxml2::XMLElement * tag = doc.NewElement(_data[iData].tag.c_str());
		tinyxml2::XMLText * value = doc.NewText(_data[iData].value.c_str());
		tag->LinkEndChild(value);
		doc.LinkEndChild(tag);
	}
	doc.SaveFile(_path.c_str());
	return true;
}

int XMLReader::getSize(){
	return _size;
}


std::string XMLReader::getPath(){
	if(_path != ""){
		return _path;
	}
	return "ERR(getPath):No Path";
}

int XMLReader::getIndex(std::string tag){
	for(int iData = 0; iData < _data.size(); iData++){
		if(_data[iData].tag == tag){
			return iData;
		}
	}
	return -1;
}

std::string XMLReader::getTag(int index){
	if(index < _size){
		return _data[index].tag;
	}
	return "ERR(getTag):Index Out Of Bounds";
}

std::string XMLReader::getValue(int index){
	if(index < _size){
		return _data[index].value;
	}
	return "ERR(getValue):Index Out Of Bounds";
}

std::string XMLReader::operator [](int index){
	if(index < _size){
		return _data[index].value;
	}
	return "ERR(getValue):Index Out Of Bounds";
}

std::string XMLReader::getValue(std::string tag){
	for(int iData = 0; iData < _data.size(); iData++){
		if(_data[iData].tag == tag){
			return _data[iData].value;
		}
	}
	return "ERR(getValue):Cannot Find Tag";
}

std::string XMLReader::operator [](std::string tag){
	for(int iData = 0; iData < _data.size(); iData++){
		if(_data[iData].tag == tag){
			return _data[iData].value;
		}
	}
	return "ERR(getValue):Cannot Find Tag";
}

bool XMLReader::setTag(std::string tag, std::string newTag){
	for(int iData = 0; iData < _data.size(); iData++){
		if(_data[iData].tag == tag){
			_data[iData].tag = newTag;
			return true;
		}
	}
	return false;
}

void XMLReader::setValue(std::string tag, std::string value){
	for(int iData = 0; iData < _data.size(); iData++){
		if(_data[iData].tag == tag){
			_data[iData].value = value;
			return;
		}
	}
	insertData(tag, value);
}

void XMLReader::setValue(int iData, std::string value){
	if(iData < _size){
		_data[iData].value = value;
	}
}

void XMLReader::insertData(std::string tag, std::string value){
	Data iData;
	iData.tag = tag;
	iData.value = value;
	_data.push_back(iData);
	_size++;
}

bool XMLReader::removeData(std::string tag){
	for(int iData = 0; iData < _data.size(); iData++){
		if(_data[iData].tag == tag){
			_data.erase(_data.begin()+iData);
			return true;
		}
	}
	return false;	
}