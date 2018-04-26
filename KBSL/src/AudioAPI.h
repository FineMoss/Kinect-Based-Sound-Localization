#include <SFML/Audio.hpp>
#include <math.h>
#include <unistd.h>
#include <vector>

using namespace std;


class Sound {

public:

	Sound();
	void scan_to_sound(vector<float>, vector<float>);
	void set_volume(float);

private:

	sf::SoundBuffer buffer;
	sf::Sound sound;
	void set_pitch(float);
	void set_position(float, float, float);
	void set_play();
	void set_pause();

};

