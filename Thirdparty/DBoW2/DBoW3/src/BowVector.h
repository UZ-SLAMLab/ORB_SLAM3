/**
 * File: BowVector.h
 * Date: March 2011
 * Author: Dorian Galvez-Lopez
 * Description: bag of words vector
 * License: see the LICENSE.txt file
 *
 */

#ifndef __D_T_BOW_VECTOR__
#define __D_T_BOW_VECTOR__

#include <map>
#include <vector>
#include <stdint.h>
 #include <string>
#include "exports.h"
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/map.hpp>
#if _WIN32
#include <cstdint>
#endif
namespace DBoW3 {

/// Id of words
typedef unsigned int WordId;

/// Value of a word
typedef double WordValue;

/// Id of nodes in the vocabulary tree
typedef unsigned int NodeId;

/// L-norms for normalization
enum LNorm
{
  L1,
  L2
};

/// Weighting type
enum WeightingType
{
  TF_IDF,
  TF,
  IDF,
  BINARY
};

/// Scoring type
enum ScoringType
{
  L1_NORM,
  L2_NORM,
  CHI_SQUARE,
  KL,
  BHATTACHARYYA,
  DOT_PRODUCT
};

/// Vector of words to represent images
class DBOW_API BowVector:
	public std::map<WordId, WordValue>
{
	friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const int version)
    {
        ar & boost::serialization::base_object<std::map<WordId, WordValue> >(*this);
    }
public:

	/** 
	 * Constructor
	 */
	BowVector(void);

	/**
	 * Destructor
	 */
	~BowVector(void);
	
	/**
	 * Adds a value to a word value existing in the vector, or creates a new
	 * word with the given value
	 * @param id word id to look for
	 * @param v value to create the word with, or to add to existing word
	 */
	void addWeight(WordId id, WordValue v);
	
	/**
	 * Adds a word with a value to the vector only if this does not exist yet
	 * @param id word id to look for
	 * @param v value to give to the word if this does not exist
	 */
	void addIfNotExist(WordId id, WordValue v);

	/**
	 * L1-Normalizes the values in the vector 
	 * @param norm_type norm used
	 */
	void normalize(LNorm norm_type);
	
	/**
	 * Prints the content of the bow vector
	 * @param out stream
	 * @param v
	 */
	friend std::ostream& operator<<(std::ostream &out, const BowVector &v);
	
	/**
	 * Saves the bow vector as a vector in a matlab file
	 * @param filename
	 * @param W number of words in the vocabulary
	 */
	void saveM(const std::string &filename, size_t W) const;

    //returns a unique number from the configuration
    uint64_t getSignature()const;
    //serialization
    void toStream(std::ostream &str)const;
    void fromStream(std::istream &str);
};

} // namespace DBoW3

#endif
