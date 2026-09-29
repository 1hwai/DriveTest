#include "AudioSystem.h"

#include <SDL3/SDL_init.h>
#include <SDL3/SDL_version.h>

#include "../Core/Debug/Logger.h"
#include "../Vehicle/Car.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

namespace {
    constexpr float Pi = 3.14159265358979323846f;
    constexpr float SampleRate = 48000.0f;
    constexpr int Channels = 2;
    constexpr int MaxFramesPerChunk = 1024;

    float Clamp01(float value) {
        return std::clamp(value, 0.0f, 1.0f);
    }

    float SineHarmonic(
        float phase,
        int harmonic
    ) {
        return std::sin(
            phase *
            static_cast<float>(harmonic)
        );
    }
}

AudioSystem::AudioSystem()
    : m_stream(nullptr),
    m_rpm(0.0f),
    m_engineTorque(0.0f),
    m_speedKmh(0.0f),
    m_maxSlip(0.0f),
    m_maxSlipAngle(0.0f),
    m_brake(0.0f),
    m_engineRunning(false),
    m_enginePhase(0.0f),
    m_exhaustPhase(0.0f),
    m_tireFilter(0.0f),
    m_noiseState(0x12345678u) {}

AudioSystem::~AudioSystem() {
    Shutdown();
}

bool AudioSystem::Initialize() {
    Logger::Debug(
        std::string("[Audio] SDL version: ") +
        std::to_string(SDL_MAJOR_VERSION) + "." +
        std::to_string(SDL_MINOR_VERSION) + "." +
        std::to_string(SDL_MICRO_VERSION)
    );

    const char* audioDriver = SDL_GetCurrentAudioDriver();

    Logger::Debug(
        std::string("[Audio] Current audio driver: ") +
        (audioDriver ? audioDriver : "<none>")
    );

    if (!SDL_InitSubSystem(SDL_INIT_AUDIO)) {
        Logger::Debug(
            std::string("[Audio] SDL audio init failed: ") +
            SDL_GetError()
        );
        return true;
    }

    const SDL_AudioSpec spec = {
        SDL_AUDIO_F32,
        Channels,
        static_cast<int>(SampleRate)
    };

    m_stream =
        SDL_OpenAudioDeviceStream(
            SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK,
            &spec,
            AudioCallback,
            this
        );

    if (!m_stream) {
        Logger::Debug(
            std::string("[Audio] Device open failed: ") +
            SDL_GetError()
        );
        SDL_QuitSubSystem(SDL_INIT_AUDIO);
        return true;
    }

    Logger::Debug(
        std::string("[Audio] Audio stream opened: ") +
        (m_stream ? "yes" : "no")
    );

    if (!SDL_ResumeAudioStreamDevice(m_stream)) {
        Logger::Debug(
            std::string("[Audio] Device resume failed: ") +
            SDL_GetError()
        );
        SDL_DestroyAudioStream(m_stream);
        m_stream = nullptr;
        SDL_QuitSubSystem(SDL_INIT_AUDIO);
        return true;
    }

    Logger::Debug(
        std::string("[Audio] Procedural vehicle audio initialized, driver=") +
        (SDL_GetCurrentAudioDriver() ?
            SDL_GetCurrentAudioDriver() :
            "<none>")
    );
    return true;
}

void AudioSystem::Update(const Car& car) {
    if (!m_stream)
        return;

    float maxSlip = 0.0f;
    float maxSlipAngle = 0.0f;
    float brake = 0.0f;

    constexpr float MaxBrakeTorque = 2500.0f;

    for (size_t i = 0; i < WheelCount; ++i) {
        const Tire& tire =
            car.GetTire(
                static_cast<WheelIndex>(i)
            );

        const Wheel& wheel =
            car.GetWheel(
                static_cast<WheelIndex>(i)
            );

        maxSlip =
            std::max(
                maxSlip,
                std::abs(tire.GetSlipRatio())
            );

        maxSlipAngle =
            std::max(
                maxSlipAngle,
                std::abs(tire.GetSlipAngle())
            );

        brake =
            std::max(
                brake,
                Clamp01(
                    wheel.GetBrakeTorque() /
                    MaxBrakeTorque
                )
            );
    }

    m_rpm.store(
        car.GetEngine().GetRPM(),
        std::memory_order_relaxed
    );

    m_engineTorque.store(
        car.GetEngine().GetTorque(),
        std::memory_order_relaxed
    );

    m_speedKmh.store(
        car.GetSpeedKmh(),
        std::memory_order_relaxed
    );

    m_maxSlip.store(
        maxSlip,
        std::memory_order_relaxed
    );

    m_maxSlipAngle.store(
        maxSlipAngle,
        std::memory_order_relaxed
    );

    m_brake.store(
        brake,
        std::memory_order_relaxed
    );

    m_engineRunning.store(
        car.GetEngine().IsRunning(),
        std::memory_order_relaxed
    );
}

void AudioSystem::Shutdown() {
    if (m_stream) {
        SDL_DestroyAudioStream(m_stream);
        m_stream = nullptr;
    }

    if (SDL_WasInit(SDL_INIT_AUDIO))
        SDL_QuitSubSystem(SDL_INIT_AUDIO);
}

void AudioSystem::AudioCallback(
    void* userdata,
    SDL_AudioStream* stream,
    int additionalAmount,
    int totalAmount
) {
    auto* audio =
        static_cast<AudioSystem*>(userdata);

    if (!audio ||
        additionalAmount <= 0)
        return;

    std::array<float, MaxFramesPerChunk * Channels> buffer{};

    static std::atomic<uint64_t> callbackCount{0};
    static std::atomic<uint64_t> submittedBytes{0};

    const uint64_t callbackIndex =
        callbackCount.fetch_add(
            1,
            std::memory_order_relaxed
        ) + 1;

    if (callbackIndex == 1 ||
        callbackIndex % 100 == 0) {
        Logger::Debug(
            std::string("[Audio] Callback count=") +
            std::to_string(callbackIndex) +
            " additionalBytes=" +
            std::to_string(additionalAmount) +
            " totalBytes=" +
            std::to_string(totalAmount)
        );
    }

    int remaining =
        additionalAmount;

    while (remaining > 0) {
        const int maxBytes =
            static_cast<int>(
                buffer.size() *
                sizeof(float)
            );

        const int bytes =
            std::min(
                remaining,
                maxBytes
            );

        const int frames =
            bytes /
            (Channels * static_cast<int>(sizeof(float)));

        if (frames <= 0)
            break;

        audio->GenerateAudio(
            buffer.data(),
            frames
        );

        const int submitted =
            frames *
            Channels *
            static_cast<int>(sizeof(float));

        const bool putResult =
            SDL_PutAudioStreamData(
                stream,
                buffer.data(),
                submitted
            );

        if (!putResult) {
            Logger::Debug(
                std::string("[Audio] SDL_PutAudioStreamData failed: ") +
                SDL_GetError()
            );
        }

        const uint64_t totalSubmitted =
            submittedBytes.fetch_add(
                static_cast<uint64_t>(submitted),
                std::memory_order_relaxed
            ) + static_cast<uint64_t>(submitted);

        if (callbackIndex == 1 ||
            (totalSubmitted / 4096u) % 100u == 0u) {
            Logger::Debug(
                std::string("[Audio] Submitted bytes=") +
                std::to_string(totalSubmitted)
            );
        }

        remaining -=
            frames *
            Channels *
            static_cast<int>(sizeof(float));
    }
}

void AudioSystem::GenerateAudio(
    float* buffer,
    int frames
) {
    const float rpm =
        m_rpm.load(std::memory_order_relaxed);

    const float engineTorque =
        m_engineTorque.load(std::memory_order_relaxed);

    const float speedKmh =
        m_speedKmh.load(std::memory_order_relaxed);

    const float maxSlip =
        m_maxSlip.load(std::memory_order_relaxed);

    const float maxSlipAngle =
        m_maxSlipAngle.load(std::memory_order_relaxed);

    const float brake =
        m_brake.load(std::memory_order_relaxed);

    const bool running =
        m_engineRunning.load(std::memory_order_relaxed);

    static std::atomic<uint64_t> generateCount{0};
    const uint64_t generateIndex =
        generateCount.fetch_add(
            1,
            std::memory_order_relaxed
        ) + 1;

    if (generateIndex == 1 ||
        generateIndex % 200 == 0) {
        Logger::Debug(
            std::string("[Audio] GenerateAudio rpm=") +
            std::to_string(rpm) +
            " torque=" +
            std::to_string(engineTorque) +
            " speed=" +
            std::to_string(speedKmh) +
            " slip=" +
            std::to_string(maxSlip) +
            " running=" +
            (running ? "1" : "0")
        );
    }

    const float rpm01 =
        Clamp01(rpm / 7000.0f);

    const float torque01 =
        Clamp01(engineTorque / 280.0f);

    const float engineGain =
        running
            ? 0.035f +
              rpm01 * 0.045f +
              torque01 * 0.075f
            : 0.0f;

    const float firingFrequency =
        std::max(rpm, 0.0f) / 30.0f;

    const float exhaustGain =
        running
            ? 0.025f +
              torque01 * 0.055f
            : 0.0f;

    const float tireSlip =
        Clamp01(
            std::max(
                maxSlip / 0.35f,
                maxSlipAngle / 0.30f
            )
        );

    const float tireSpeed =
        Clamp01(speedKmh / 120.0f);

    const float tireGain =
        tireSlip *
        tireSpeed *
        (0.015f + brake * 0.025f);

    for (int i = 0; i < frames; ++i) {
        float engine = 0.0f;
        float exhaust = 0.0f;

        if (running) {
            engine =
                SineHarmonic(
                    m_enginePhase,
                    1
                ) * 0.55f +
                SineHarmonic(
                    m_enginePhase,
                    2
                ) * 0.22f +
                SineHarmonic(
                    m_enginePhase,
                    3
                ) * 0.12f +
                SineHarmonic(
                    m_enginePhase,
                    5
                ) * 0.07f;

            exhaust =
                SineHarmonic(
                    m_exhaustPhase,
                    1
                ) * 0.65f +
                SineHarmonic(
                    m_exhaustPhase,
                    2
                ) * 0.25f;

            m_enginePhase +=
                2.0f *
                Pi *
                firingFrequency /
                SampleRate;

            m_exhaustPhase +=
                2.0f *
                Pi *
                firingFrequency *
                0.5f /
                SampleRate;

            if (m_enginePhase >= 2.0f * Pi)
                m_enginePhase -= 2.0f * Pi;

            if (m_exhaustPhase >= 2.0f * Pi)
                m_exhaustPhase -= 2.0f * Pi;
        }

        m_noiseState =
            m_noiseState * 1664525u +
            1013904223u;

        const float noise =
            (
                static_cast<float>(
                    (m_noiseState >> 8) & 0x00ffffffu
                ) /
                8388607.5f
            ) - 1.0f;

        const float tireNoise =
            noise * 0.75f +
            m_tireFilter * 0.25f;

        m_tireFilter =
            tireNoise;

        const float sample =
            engine * engineGain +
            exhaust * exhaustGain +
            tireNoise * tireGain;

        const float limited =
            std::tanh(sample * 1.8f) * 0.75f;

        buffer[i * 2 + 0] = limited;
        buffer[i * 2 + 1] = limited;
    }
}
