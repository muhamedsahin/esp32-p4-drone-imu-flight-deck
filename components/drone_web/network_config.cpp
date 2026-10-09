#include "network_config.hpp"
#include <array>
#include <charconv>

namespace drone_web
{
    namespace
    {
        std::string_view trim(std::string_view value)
        {
            const auto first = value.find_first_not_of(" \t\r\n");
            if (first == std::string_view::npos)
                return {};
            const auto last = value.find_last_not_of(" \t\r\n");
            return value.substr(first, last - first + 1);
        }
        bool quotedValue(std::string_view raw, std::string &value)
        {
            if (raw == "null")
            {
                value.clear();
                return true;
            }
            if (raw.size() < 2 || raw.front() != '"' || raw.back() != '"')
                return false;
            raw.remove_prefix(1);
            raw.remove_suffix(1);
            // Kaçış dizisi yok: dosyadaki ters bölü gerçek şifre karakteridir.
            for (unsigned char c : raw)
                if (c < 32 || c == 127 || c == '"')
                    return false;
            value.assign(raw);
            return true;
        }
        bool passwordValid(const std::string &value)
        {
            if (value.size() < 8 || value.size() > 63)
                return false;
            for (unsigned char c : value)
                if (c < 32 || c > 126)
                    return false;
            return true;
        }
    }
    bool parseNetworkConfig(std::string_view text, NetworkConfig &output, std::string &error)
    {
        NetworkConfig candidate;
        std::array<bool, 5> seen{};
        constexpr std::array<std::string_view, 5> keys{
            "ssid", "password", "ap_ssid", "ap_password", "connect_timeout_s"};
        error.clear();
        // UTF-8 BOM ve Windows CRLF desteklenir.
        if (text.substr(0, 3) == "\xEF\xBB\xBF")
            text.remove_prefix(3);
        while (!text.empty())
        {
            auto end = text.find('\n');
            auto line = trim(text.substr(0, end));
            text = end == std::string_view::npos ? std::string_view{} : text.substr(end + 1);
            bool quoted = false;
            for (size_t i = 0; i < line.size(); ++i)
            {
                if (line[i] == '"')
                    quoted = !quoted;
                if (line[i] == '#' && !quoted)
                {
                    line = trim(line.substr(0, i));
                    break;
                }
            }
            if (line.empty())
                continue;
            auto separator = line.find('=');
            if (separator == std::string_view::npos)
            {
                error = "Anahtar = deger bekleniyor.";
                return false;
            }
            auto key = trim(line.substr(0, separator));
            auto raw = trim(line.substr(separator + 1));
            size_t index = 0;
            while (index < keys.size() && keys[index] != key)
                ++index;
            if (index == keys.size() || seen[index])
            {
                error = "Bilinmeyen veya tekrar eden anahtar.";
                return false;
            }
            seen[index] = true;
            if (index == 4)
            {
                unsigned seconds = 0;
                const auto result = std::from_chars(raw.data(), raw.data() + raw.size(), seconds);
                if (result.ec != std::errc{} || result.ptr != raw.data() + raw.size() || seconds < 5 || seconds > 120)
                {
                    error = "connect_timeout_s 5-120 olmali.";
                    return false;
                }
                candidate.connect_timeout_s = seconds;
            }
            else
            {
                std::string *values[]{&candidate.ssid, &candidate.password, &candidate.ap_ssid, &candidate.ap_password};
                if (!quotedValue(raw, *values[index]))
                {
                    error = "Metin cift tirnakli veya null olmali.";
                    return false;
                }
            }
        }
        if (!seen[0] || !seen[1])
        {
            error = "ssid ve password alanlari gerekli.";
            return false;
        }
        if (candidate.ssid.size() > 32 || candidate.ap_ssid.empty() || candidate.ap_ssid.size() > 32)
        {
            error = "SSID uzunlugu 1-32 bayt olmali (STA null olabilir).";
            return false;
        }
        if ((!candidate.password.empty() && !passwordValid(candidate.password)) || !passwordValid(candidate.ap_password))
        {
            error = "Sifre 8-63 ASCII karakter olmali.";
            return false;
        }
        output = candidate;
        return true;
    }
}
