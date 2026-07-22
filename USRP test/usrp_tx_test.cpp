#include <uhd/stream.hpp>
#include <uhd/types/metadata.hpp>
#include <uhd/usrp/multi_usrp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {
constexpr double pi = 3.14159265358979323846;

struct Options {
    std::string args = "type=b200";
    double frequency = 2.4e9;
    double rate = 5e6;
    double gain = 0.0;
    double tone = 1e6;
    double amplitude = 0.1;
    double seconds = 10.0;
};

void usage(const char* name) {
    std::cout
        << "Usage: " << name << " [options]\n"
        << "  --args STRING       UHD device args (default: type=b200)\n"
        << "  --freq HZ           RF centre frequency (default: 2.4e9)\n"
        << "  --rate SAMP_PER_S   sample rate (default: 5e6)\n"
        << "  --gain DB           TX gain (default: 0)\n"
        << "  --tone HZ           tone offset from centre (default: 1e6)\n"
        << "  --amplitude VALUE   digital amplitude, 0..0.5 (default: 0.1)\n"
        << "  --seconds VALUE     transmit duration (default: 10)\n";
}

Options parse_options(int argc, char** argv) {
    Options opt;
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        if (key == "--help" || key == "-h") {
            usage(argv[0]);
            std::exit(0);
        }
        if (i + 1 >= argc) {
            throw std::runtime_error("Missing value for " + key);
        }
        const std::string value = argv[++i];
        if (key == "--args") opt.args = value;
        else if (key == "--freq") opt.frequency = std::stod(value);
        else if (key == "--rate") opt.rate = std::stod(value);
        else if (key == "--gain") opt.gain = std::stod(value);
        else if (key == "--tone") opt.tone = std::stod(value);
        else if (key == "--amplitude") opt.amplitude = std::stod(value);
        else if (key == "--seconds") opt.seconds = std::stod(value);
        else throw std::runtime_error("Unknown option: " + key);
    }
    if (opt.rate <= 0.0 || opt.seconds <= 0.0)
        throw std::runtime_error("Rate and duration must be positive");
    if (opt.amplitude <= 0.0 || opt.amplitude > 0.5)
        throw std::runtime_error("Amplitude must be greater than 0 and at most 0.5");
    if (std::abs(opt.tone) >= opt.rate / 2.0)
        throw std::runtime_error("Tone must be inside +/- sample_rate/2");
    return opt;
}
} // namespace

int main(int argc, char** argv) {
    try {
        const Options opt = parse_options(argc, argv);
        std::cout << "Opening USRP with args: " << opt.args << '\n';
        auto usrp = uhd::usrp::multi_usrp::make(opt.args);

        usrp->set_tx_rate(opt.rate, 0);
        usrp->set_tx_freq(uhd::tune_request_t(opt.frequency), 0);
        usrp->set_tx_gain(opt.gain, 0);
        usrp->set_tx_bandwidth(std::min(opt.rate, 5e6), 0);

        const double actual_rate = usrp->get_tx_rate(0);
        const double actual_freq = usrp->get_tx_freq(0);
        const double actual_gain = usrp->get_tx_gain(0);
        std::cout << "TX channel 0: centre=" << actual_freq << " Hz, rate="
                  << actual_rate << " S/s, gain=" << actual_gain << " dB\n"
                  << "Expected analyzer tone: " << actual_freq + opt.tone << " Hz\n";

        uhd::stream_args_t stream_args("fc32", "sc16");
        stream_args.channels = {0};
        auto streamer = usrp->get_tx_stream(stream_args);
        const std::size_t count = std::max<std::size_t>(1024, streamer->get_max_num_samps());
        std::vector<std::complex<float>> samples(count);
        double phase = 0.0;
        const double phase_step = 2.0 * pi * opt.tone / actual_rate;

        uhd::tx_metadata_t md;
        md.start_of_burst = true;
        md.end_of_burst = false;
        md.has_time_spec = false;

        const auto stop = std::chrono::steady_clock::now()
                        + std::chrono::duration<double>(opt.seconds);
        while (std::chrono::steady_clock::now() < stop) {
            for (auto& sample : samples) {
                sample = static_cast<float>(opt.amplitude)
                       * std::complex<float>(std::cos(phase), std::sin(phase));
                phase += phase_step;
                if (phase > pi) phase -= 2.0 * pi;
                else if (phase < -pi) phase += 2.0 * pi;
            }
            streamer->send(samples.data(), samples.size(), md, 1.0);
            md.start_of_burst = false;
        }

        md.end_of_burst = true;
        streamer->send(static_cast<const std::complex<float>*>(nullptr), 0, md, 1.0);
        std::cout << "Transmission complete.\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "USRP test failed: " << e.what() << '\n';
        return 1;
    }
}
