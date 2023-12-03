class Program
{
    public static void Main()
    {
        Quaternion quaternion1 = new Quaternion(1, 2, 3, 4);
        Quaternion quaternion2 = new Quaternion(1, 2, 3, 4);
        Quaternion quaternion3 = new Quaternion(2, 3, 4, 5);

        if (quaternion1 == quaternion2)
        {
            Console.WriteLine($"{quaternion1} equals {quaternion2}\n");
        }
        else
        {
            Console.WriteLine($"{quaternion1} does not equal {quaternion2}\n");
        }

        if (quaternion2 != quaternion3)
        {
            Console.WriteLine($"{quaternion2} does not equal {quaternion3}\n");
        }
        else
        {
            Console.WriteLine($"{quaternion2} equals {quaternion3}\n");
        }

        Console.WriteLine($"({quaternion1}) + ({quaternion3}) = {quaternion1 + quaternion3}\n" +
                          $"({quaternion1}) - ({quaternion3}) = {quaternion1 - quaternion3}\n" +
                          $"({quaternion1}) * ({quaternion3}) = {quaternion1 * quaternion3}\n" +
                          $"({quaternion1}) / ({quaternion3}) = {quaternion1 / quaternion3}\n");

        Console.WriteLine($"Norm: {quaternion1.Norm()}\n" +
                          $"Conjugate: {quaternion1.Conjugate()}\n" +
                          $"Inverse: {quaternion1.Inverse()}\n");

        double[,] rotationMatrix = quaternion1.ToRotationMatrix();
        Console.WriteLine("Quaternion: " + quaternion1);
        Console.WriteLine("Rotation Matrix:");
        PrintMatrix(rotationMatrix);

        Quaternion qFromMatrix = Quaternion.FromRotationMatrix(rotationMatrix);
        Console.WriteLine("Quaternion from Rotation Matrix: " + qFromMatrix);
    }

    static void PrintMatrix(double[,] matrix)
    {
        for (int i = 0; i < matrix.GetLength(0); i++)
        {
            for (int j = 0; j < matrix.GetLength(1); j++)
            {
                Console.Write(matrix[i, j] + " ");
            }
            Console.WriteLine();
        }
    }
}
