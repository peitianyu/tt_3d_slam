#ifndef __PLATFORM_SERIALIZE_H__
#define __PLATFORM_SERIALIZE_H__

#include <sstream> 
#include <vector>

namespace platform
{

class Serialize
{
public:
    Serialize() : m_oss(std::ios::binary | std::ios::out) {}

    void Reset(){ m_oss.str("");}

    template <typename T>
    Serialize& operator<<(const T& t)
    {
        m_oss.write(reinterpret_cast<const char*>(&t), sizeof(T));
        return *this;
    }

    template <typename T>
    Serialize& operator<<(const std::vector<T>& t)
    {
        size_t size = t.size();
        m_oss.write(reinterpret_cast<const char*>(&size), sizeof(size_t));
        for (size_t i = 0; i < size; i++)
            m_oss.write(reinterpret_cast<const char*>(&t[i]), sizeof(T));
        return *this;
    }

    std::string str() const { return m_oss.str(); }

private:
    std::ostringstream m_oss;
};

} // namespace platform

#endif // __PLATFORM_SERIALIZE_H__