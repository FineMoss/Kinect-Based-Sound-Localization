	
#include <AudioAPI.h>

	// initializes the sound object
	// a sin wave is stored in the sound buffer
	// the sound is on a loop
	// volume is set to 0 
	Sound::Sound() {
		const unsigned SAMPLES     = 44100;
		const unsigned SAMPLE_RATE = 44100;
		const unsigned AMPLITUDE   = 30000;
		const double TWO_PI        = 6.28318;
		const double increment     = 440./44100;
		sf::Int16 raw[SAMPLES];
		double x = 0;

		for (unsigned i = 0; i < SAMPLES; i++) {
			raw[i] = AMPLITUDE * sin(x*TWO_PI);
			x += increment;
		}
		
		if (!buffer.loadFromSamples(raw, SAMPLES, 1, SAMPLE_RATE)) {
			// error
		}

		sound.setBuffer(buffer);
		sound.setLoop(true);
		sound.setVolume(0.f);
		sound.play();

		sf::Listener::setPosition(0.f, 0.f, 0.f);
		sf::Listener::setDirection(1.f, 0.f, 0.f);
		sf::Listener::setGlobalVolume(0.f);

	}

	// changes the volume of the sound object
	// takes a float as parameter
	// volume ranges from 0-100
	void Sound::set_volume(float volume) {
		sound.setVolume(volume);
		sf::Listener::setGlobalVolume(volume);
	}

	// changes the pitch of the sound object
	// takes a float as parameter
	// 1.0 will bring it back to the original sound
	void Sound::set_pitch(float pitch) {
		sound.setPitch(pitch);
	}

	// sets the position of the sound object
	// takes three floats as parameters
	// +x is in front of the listener
	// +y is above the listener
	// +z is to the right of the listener
	void Sound::set_position(float x, float y, float z) {
		sound.setPosition(x, y, z);
	}

	// resumes playing the sound object
	void Sound::set_play() {
		sound.play();
	}

	// pauses the sound object
	void Sound::set_pause() {
		sound.pause();
	}


	// angle -0.7, +0.7 in rad
	// distance in meters
	void Sound::scan_to_sound(vector<float> angle, vector<float> distance) {	
		int max_index = 0;
		float max_distance = 0;
		for (int i = 0; i < distance.size(); i++) {
			if (distance.at(i) > max_distance) {
				max_index = i;
				max_distance = distance.at(i);
			}
		}
		if (max_index == 0) {
			sound.setPosition(0.f, 0.f, 10.f);
			sound.setVolume(100.f);
			sf::Listener::setGlobalVolume(100.f);
		}
		else if(max_index == 1) {
			sound.setPosition(0.f, 0.f, 5.f);
			sound.setVolume(50.f);
			sf::Listener::setGlobalVolume(50.f);
		}
		else if(max_index == 2) {
			sound.setPosition(0.f, 0.f, 0.f);
			sound.setVolume(0.f);
			sf::Listener::setGlobalVolume(0.f);
		}
		else if(max_index == 3) {
			sound.setPosition(0.f, 0.f, -5.f);
			sound.setVolume(50.f);
			sf::Listener::setGlobalVolume(50.f);
		}
		else if(max_index == 4) {
			sound.setPosition(0.f, 0.f, -10.f);
			sound.setVolume(100.f);
			sf::Listener::setGlobalVolume(100.f);
		}
		else {
			//error
			sf::Listener::setPosition(0.f, 0.f, 0.f);
			sound.setVolume(0.f);
			sf::Listener::setGlobalVolume(0.f);
		}

	}