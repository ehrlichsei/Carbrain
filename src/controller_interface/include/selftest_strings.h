#ifndef SELFTEST_STRINGS
#define SELFTEST_STRINGS

#include <string>
#include <vector>
#include <map>

typedef std::vector<std::string> StringVector;
typedef std::map<int, std::string> StringMap;
typedef std::map<int, StringVector> StringVectorMap;


namespace SelftestStrings
{

StringMap createNameMap();
StringMap createMessageMap();
StringVectorMap createKeyVectorMap();

static const StringMap names = createNameMap();
static const StringMap messages = createMessageMap();
static const StringVectorMap keys = createKeyVectorMap();

}


#endif //SELFTEST_STRINGS
