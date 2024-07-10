#ifndef __AA_KROMEK_HPP__
#define __AA_KROMEK_HPP__

#include <map>
#include <iostream>
#include <string_view>
#include <vector>
#include <memory>
#include <tuple>

namespace Kromek {

class Sensor {
public:
    using Spectrum = std::vector<unsigned>;
    explicit Sensor(unsigned id);
    ~Sensor();
    // Acquisition
    bool aquiring() const;
    void start(bool accumulate = false);
    void stop();
    void clear();
    // Data
    std::tuple<unsigned, Spectrum> data() const;
    unsigned counts() const;
    Spectrum spectrum() const;
    unsigned realtime() const;
    unsigned livetime() const;
    // Info
    unsigned channels() const;
    unsigned id() const { return m_id; }
    int serial() const { return m_serial; }
    std::string name() const;
    std::string manufacturer() const;
    int vendor() const;
    int product() const;
    std::string info() const;
private:
    unsigned m_id;
    unsigned m_serial;
    // Acquistion
    unsigned m_realtime;
    unsigned m_livetime;
};

class Driver {
public:
    using Devices = std::map<unsigned, std::shared_ptr<Sensor>>;
    using Iterator = Devices::iterator;
    explicit Driver();
    ~Driver();
    std::string version() const;
    // Devices
    std::shared_ptr<Sensor> device(unsigned serial);
    const std::shared_ptr<Sensor> device(unsigned serial) const { return device(serial); }
    Devices &devices() { return s_devices; }
    const Devices &devices() const { return s_devices; }
protected:
    static Iterator findDevice(unsigned id);
    static void clearDevices();
    static void addDevice(unsigned id);
    static void removeDevice(unsigned id);
    static void deviceChangedCallback(unsigned id, int added, void *caller);
    static void errorCallback(void *caller, unsigned id, int code, const char *message);
private:
    // Devices
    static Devices s_devices; // {serial, sensor}
};

}

#endif