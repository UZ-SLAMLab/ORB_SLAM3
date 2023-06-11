/**
 * File: BowVector.cpp
 * Date: March 2011
 * Author: Dorian Galvez-Lopez
 * Description: bag of words vector
 * License: see the LICENSE.txt file
 *
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <cmath>

#include "BowVector.h"

namespace DBoW3 {

// --------------------------------------------------------------------------

BowVector::BowVector(void)
{
}

// --------------------------------------------------------------------------

BowVector::~BowVector(void)
{
}

// --------------------------------------------------------------------------

void BowVector::addWeight(WordId id, WordValue v)
{
  BowVector::iterator vit = this->lower_bound(id);
  
  if(vit != this->end() && !(this->key_comp()(id, vit->first)))
  {
    vit->second += v;
  }
  else
  {
    this->insert(vit, BowVector::value_type(id, v));
  }
}

// --------------------------------------------------------------------------

void BowVector::addIfNotExist(WordId id, WordValue v)
{
  BowVector::iterator vit = this->lower_bound(id);
  
  if(vit == this->end() || (this->key_comp()(id, vit->first)))
  {
    this->insert(vit, BowVector::value_type(id, v));
  }
}

// --------------------------------------------------------------------------

void BowVector::normalize(LNorm norm_type)
{
  double norm = 0.0; 
  BowVector::iterator it;

  if(norm_type == DBoW3::L1)
  {
    for(it = begin(); it != end(); ++it)
      norm += fabs(it->second);
  }
  else
  {
    for(it = begin(); it != end(); ++it)
      norm += it->second * it->second;
		norm = sqrt(norm);  
  }

  if(norm > 0.0)
  {
    for(it = begin(); it != end(); ++it)
      it->second /= norm;
  }
}

// --------------------------------------------------------------------------

std::ostream& operator<< (std::ostream &out, const BowVector &v)
{
  BowVector::const_iterator vit;
  std::vector<unsigned int>::const_iterator iit;
  unsigned int i = 0; 
  const size_t N = v.size();
  for(vit = v.begin(); vit != v.end(); ++vit, ++i)
  {
    out << "<" << vit->first << ", " << vit->second << ">";
    
    if(i < N-1) out << ", ";
  }
  return out;
}

// --------------------------------------------------------------------------

void BowVector::saveM(const std::string &filename, size_t W) const
{
  std::fstream f(filename.c_str(), std::ios::out);
  
  WordId last = 0;
  BowVector::const_iterator bit;
  for(bit = this->begin(); bit != this->end(); ++bit)
  {
    for(; last < bit->first; ++last)
    {
      f << "0 ";
    }
    f << bit->second << " ";
    
    last = bit->first + 1;
  }
  for(; last < (WordId)W; ++last)
    f << "0 ";
  
  f.close();
}
// --------------------------------------------------------------------------

void BowVector::toStream(std::ostream &str)const{
    uint32_t s=size();
    str.write((char*)&s,sizeof(s));
    for(auto d:*this){
        str.write((char*)&d.first,sizeof(d.first));
        str.write((char*)&d.second,sizeof(d.second));
    }
}
// --------------------------------------------------------------------------

void BowVector::fromStream(std::istream &str){
clear();
uint32_t s;

str.read((char*)&s,sizeof(s));
for(int i=0;i<s;i++){
    WordId wid;
    WordValue wv;
    str.read((char*)&wid,sizeof(wid));
    str.read((char*)&wv,sizeof(wv));
    insert(std::make_pair(wid,wv));
}

}

uint64_t BowVector::getSignature()const{
uint64_t sig=0;
for(auto ww:*this) sig+=ww.first+1e6*ww.second;
return sig;
}


// --------------------------------------------------------------------------

} // namespace DBoW3

