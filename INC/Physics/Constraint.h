#pragma once


namespace Physebs {
	enum eConstraint { SPRING, JOINT };

	class Rigidbody;

	class Constraint {
	public:
		Constraint(Rigidbody* a_attachedActor, Rigidbody* a_attachedOther);
		virtual ~Constraint();

		virtual void Update();
		virtual void Constrain() = 0;

		virtual void Draw() = 0;

		eConstraint GetType() const { return m_type; }
	protected:
		Rigidbody* m_attachedActor;
		Rigidbody* m_attachedOther;

		eConstraint m_type;
	private:
	};
}
