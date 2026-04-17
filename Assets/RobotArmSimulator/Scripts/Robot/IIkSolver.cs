namespace RobotArmSimulator
{
    public interface IIkSolver
    {
        void Solve(float deltaTime);
        void SolveImmediately(int iterations = 36);
    }
}
