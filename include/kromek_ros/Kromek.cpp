#include "Kromek.hpp"
#include "Helpers.hpp"
#include <SpectrometerDriver.h>
#include <sstream>
#include <format>

const size_t g_size = 126;  // Max number of characters for a USB product.
char g_buffer[g_size];

unsigned fromId(unsigned id) {
    int bytes;
    kr_GetDeviceSerial(id, g_buffer, g_size, &bytes);
    return std::stoi(std::string(g_buffer, bytes));
}

//////////////////////////////////////////////////////////////
// Sensor

Kromek::Sensor::Sensor(unsigned id) 
: m_id(id), 
  m_serial(fromId(id)),
  m_realtime(0),
  m_livetime(0) {
    log() << "Kromek: Sensor " << info() << " connected!" << std::endl;
}

Kromek::Sensor::~Sensor() {
    log() << "Kromek: Sensor " << info() << " desconnected!" << std::endl;
}

bool Kromek::Sensor::aquiring() const { 
    return kr_IsAcquiringData(m_id); 
}

void Kromek::Sensor::start(bool accumulate) {
    if (aquiring())
        stop();
    if (!accumulate)
        clear();
    auto okay = kr_BeginDataAcquisition(m_id, m_realtime*1000, m_livetime*1000);
    if ( okay != ERROR_OK)
        log() << "Kromek error: Ops, unable to read from device";
}

void Kromek::Sensor::stop() {
    kr_StopDataAcquisition(m_id);
}

void Kromek::Sensor::clear() {
    kr_ClearAcquiredData(m_id);
}

std::tuple<unsigned, Kromek::Sensor::Spectrum> Kromek::Sensor::data() const {
    Spectrum spectrum(channels());
    unsigned counts = 0;
    kr_GetAcquiredData(m_id, spectrum.data(), &counts, nullptr, nullptr);
    return {counts, spectrum};
}

unsigned Kromek::Sensor::counts() const {
    unsigned counts = 0;
    kr_GetAcquiredData(m_id, nullptr, &counts, nullptr, nullptr);
    return counts;
}

Kromek::Sensor::Spectrum Kromek::Sensor::spectrum() const {
    Spectrum spectrum(channels());
    kr_GetAcquiredData(m_id, spectrum.data(), nullptr, nullptr, nullptr);
    return spectrum;
}

unsigned Kromek::Sensor::realtime() const {
    unsigned time = 0;
    kr_GetAcquiredData(m_id, nullptr, nullptr, &time, nullptr);
    return time;
}

unsigned Kromek::Sensor::livetime() const {
    unsigned time = 0;
    kr_GetAcquiredData(m_id, nullptr, nullptr, nullptr, &time);
    return time;
}

unsigned Kromek::Sensor::channels() const {
    return TOTAL_RESULT_CHANNELS;
}

std::string Kromek::Sensor::name() const {
    int bytes;
    kr_GetDeviceName(m_id, g_buffer, g_size, &bytes);
    return std::string(g_buffer, bytes);
}

std::string Kromek::Sensor::manufacturer() const {
    int bytes;
    kr_GetDeviceManufacturer(m_id, g_buffer, g_size, &bytes);
    return std::string(g_buffer, bytes);
}

int Kromek::Sensor::vendor() const {
    int value;
    kr_GetDeviceVendorID(m_id, &value);
    return value;
}

int Kromek::Sensor::product() const {
    int value;
    kr_GetDeviceProductID(m_id, &value);
    return value;
}

std::string Kromek::Sensor::info() const {
    return std::format("[{}] {} SN#{}", id(), name(), serial());
}

//////////////////////////////////////////////////////////////
// Driver

Kromek::Driver::Devices Kromek::Driver::s_devices;

Kromek::Driver::Driver() {
    log() << "Opening Kromek driver, version " + version() << std::endl;
    kr_SetDeviceChangedCallback(deviceChangedCallback, this);
    kr_Initialise(errorCallback, this);
}

Kromek::Driver::~Driver() {
    clearDevices();
    kr_Destruct();
    log() << "Closing kromek driver." << std::endl;
}

std::string Kromek::Driver::version() const {
    int product, major, minor, build;
    kr_GetVersionInformation(&product, &major, &minor, &build);
    return std::format("{}-{}.{}.{}", product, major, minor, build);
}

std::shared_ptr<Kromek::Sensor> Kromek::Driver::device(unsigned serial) {
    auto s = s_devices.find(serial);
    if (s == s_devices.end())
        return nullptr;
    else
        return s->second;
}

Kromek::Driver::Iterator Kromek::Driver::findDevice(unsigned id) {
    for (auto i = s_devices.begin(); i != s_devices.end(); ++i) {
        if (i->second->id() == id) {
            return i;
        }
    }
    return s_devices.end();
}

void Kromek::Driver::clearDevices() {
    s_devices.clear();
}

void Kromek::Driver::addDevice(unsigned id) {
    removeDevice(id);
    auto s = std::make_shared<Sensor>(id);
    s_devices.insert({s->serial(), s});
}

void Kromek::Driver::removeDevice(unsigned id) {
    auto i = findDevice(id);
    if (i != s_devices.end())
        s_devices.erase(i);
}

//////////////////////////////////////////////////////////////
// Driver Callbacks

void Kromek::Driver::deviceChangedCallback(unsigned id, BOOL added, void *caller) {  
    if (added)
        addDevice(id);
    else
        removeDevice(id);
}
    
void Kromek::Driver::errorCallback(void *caller, unsigned id, int code, const char *message) {
    log() << "Kromek error: Ops, ";
    switch (code) {
        case ERROR_NOT_INITIALISED:
            log() << "library not initialised!";
            break;
        case ERROR_ACQUISITION_COMPLETE:
            log() << "acquisition completed!";
            break;
        default:
            log() << "Unknown message" << message;
    }
}