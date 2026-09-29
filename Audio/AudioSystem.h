#pragma once

#include <SDL3/SDL_audio.h>

#include <atomic>
#include <cstdint>

class Car;

class AudioSystem {
public:
    AudioSystem();
    ~AudioSystem();

    bool Initialize();
    void Update(const Car& car);
    void Shutdown();

private:
    static void AudioCallback(
        void* userdata,
        SDL_AudioStream* stream,
        int additionalAmount,
        int totalAmount
    );

    void GenerateAudio(
        float* buffer,
        int frames
    );

private:
    SDL_AudioStream* m_stream;

    std::atomic<float> m_rpm;
    std::atomic<float> m_engineTorque;
    std::atomic<float> m_speedKmh;
    std::atomic<float> m_maxSlip;
    std::atomic<float> m_maxSlipAngle;
    std::atomic<float> m_brake;
    std::atomic<bool> m_engineRunning;

    float m_enginePhase;
    float m_exhaustPhase;
    float m_tireFilter;
    uint32_t m_noiseState;
};
