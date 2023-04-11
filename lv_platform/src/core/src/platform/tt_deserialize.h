#ifndef __PLATFORM_DESERIALIZE_H__
#define __PLATFORM_DESERIALIZE_H__

#include <sstream> 
#include <vector>

namespace platform
{
class Deserialize
{
public:
    Deserialize(const std::string& str) : m_str(str), m_pos(0) {}

    template <typename T>
    Deserialize& operator>>(T& t)
    {
        t = *reinterpret_cast<const T*>(m_str.data() + m_pos);
        m_pos += sizeof(T);
        return *this;
    }

    template <typename T>
    Deserialize& operator>>(std::vector<T>& t)
    {
        size_t size = *reinterpret_cast<const size_t*>(m_str.data() + m_pos);
        m_pos += sizeof(size_t);
        t.resize(size);
        for (size_t i = 0; i < size; i++)
        {
            t[i] = *reinterpret_cast<const T*>(m_str.data() + m_pos);
            m_pos += sizeof(T);
        }
        return *this;
    }
private:
    const std::string& m_str;
    size_t m_pos;
};

} // namespace platform

#endif // __PLATFORM_DESERIALIZE_H__