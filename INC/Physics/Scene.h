#pragma once

#include <vector>

namespace Physebs {
	class Rigidbody;

	class Scene {
	public:
		Scene();
		~Scene();

		void Update(float a_dt);

		void AddObject(Rigidbody* a_obj);
		void RemoveObject(Rigidbody* a_obj);
	protected:
		std::vector<Rigidbody*> m_objects;
	private:
	};
}